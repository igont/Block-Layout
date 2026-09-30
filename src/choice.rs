//! Детерминированный конечный выбор совместимых вариантов узлов.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Variable {
    pub id: String,
    pub values: Vec<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct UnaryConstraint {
    pub variable: String,
    pub allowed: Vec<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BinaryConstraint {
    pub left: String,
    pub right: String,
    pub allowed: Vec<(String, String)>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Problem {
    pub variables: Vec<Variable>,
    pub unary: Vec<UnaryConstraint>,
    pub binary: Vec<BinaryConstraint>,
    pub max_states: usize,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Solution {
    pub assignment: BTreeMap<String, String>,
    pub states_examined: usize,
}

/// Первая доказанно несовместимая связная компонента бинарного графа.
/// Индексы ограничений относятся к исходному `Problem`.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ConflictComponent {
    pub variables: Vec<String>,
    pub unary_constraint_indices: Vec<usize>,
    pub binary_constraint_indices: Vec<usize>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum SolveError {
    Invalid(String),
    Unsat,
    Exhausted {
        budget: usize,
        states_examined: usize,
    },
}

struct Table {
    left: usize,
    right: usize,
    allowed: BTreeSet<(usize, usize)>,
}

struct Compiled {
    ids: Vec<String>,
    values: Vec<Vec<String>>,
    tables: Vec<Table>,
    arcs: Vec<(usize, bool)>,
    incoming_arcs: Vec<Vec<usize>>,
}

fn compile(problem: &Problem) -> Result<(Compiled, Vec<Vec<usize>>), SolveError> {
    if problem.max_states == 0 {
        return Err(SolveError::Invalid("max_states must be positive".into()));
    }
    let mut ordered = BTreeMap::<String, Vec<String>>::new();
    for variable in &problem.variables {
        if variable.id.is_empty() || ordered.contains_key(&variable.id) {
            return Err(SolveError::Invalid(format!(
                "empty or duplicate variable id: {}",
                variable.id
            )));
        }
        let mut values = variable.values.clone();
        values.sort();
        if values.iter().any(String::is_empty) || values.windows(2).any(|pair| pair[0] == pair[1]) {
            return Err(SolveError::Invalid(format!(
                "empty or duplicate value id for {}",
                variable.id
            )));
        }
        ordered.insert(variable.id.clone(), values);
    }
    let ids = ordered.keys().cloned().collect::<Vec<_>>();
    let values = ordered.into_values().collect::<Vec<_>>();
    let indexes = ids
        .iter()
        .enumerate()
        .map(|(i, id)| (id.as_str(), i))
        .collect::<BTreeMap<_, _>>();
    let mut domains = values
        .iter()
        .map(|row| (0..row.len()).collect::<Vec<_>>())
        .collect::<Vec<_>>();
    for unary in &problem.unary {
        let Some(&index) = indexes.get(unary.variable.as_str()) else {
            return Err(SolveError::Invalid(format!(
                "unknown unary variable: {}",
                unary.variable
            )));
        };
        let allowed = unary
            .allowed
            .iter()
            .map(|name| {
                values[index].binary_search(name).map_err(|_| {
                    SolveError::Invalid(format!(
                        "unknown unary value {} for {}",
                        name, unary.variable
                    ))
                })
            })
            .collect::<Result<BTreeSet<_>, _>>()?;
        domains[index].retain(|option| allowed.contains(option));
    }
    let mut tables = Vec::new();
    for binary in &problem.binary {
        let Some(&left) = indexes.get(binary.left.as_str()) else {
            return Err(SolveError::Invalid(format!(
                "unknown binary variable: {}",
                binary.left
            )));
        };
        let Some(&right) = indexes.get(binary.right.as_str()) else {
            return Err(SolveError::Invalid(format!(
                "unknown binary variable: {}",
                binary.right
            )));
        };
        if left == right {
            return Err(SolveError::Invalid(format!(
                "self constraint: {}",
                binary.left
            )));
        }
        let mut allowed = BTreeSet::new();
        for (a, b) in &binary.allowed {
            let a = values[left].binary_search(a).map_err(|_| {
                SolveError::Invalid(format!("unknown left value for {}", binary.left))
            })?;
            let b = values[right].binary_search(b).map_err(|_| {
                SolveError::Invalid(format!("unknown right value for {}", binary.right))
            })?;
            allowed.insert((a, b));
        }
        tables.push(Table {
            left,
            right,
            allowed,
        });
    }
    // Сортировка таблиц устраняет влияние порядка передачи ограничений.
    tables.sort_by_key(|table| (table.left, table.right));
    let arcs = (0..tables.len())
        .flat_map(|index| [(index, false), (index, true)])
        .collect();
    let mut incoming_arcs = vec![Vec::new(); ids.len()];
    for (index, table) in tables.iter().enumerate() {
        incoming_arcs[table.right].push(index * 2);
        incoming_arcs[table.left].push(index * 2 + 1);
    }
    Ok((
        Compiled {
            ids,
            values,
            tables,
            arcs,
            incoming_arcs,
        },
        domains,
    ))
}

fn revise(compiled: &Compiled, domains: &mut [Vec<usize>], arc: (usize, bool)) -> Option<usize> {
    let table = &compiled.tables[arc.0];
    let (source, target) = if arc.1 {
        (table.right, table.left)
    } else {
        (table.left, table.right)
    };
    let before = domains[source].len();
    let target_values = domains[target].clone();
    domains[source].retain(|&candidate| {
        target_values.iter().any(|&other| {
            let pair = if arc.1 {
                (other, candidate)
            } else {
                (candidate, other)
            };
            table.allowed.contains(&pair)
        })
    });
    (domains[source].len() != before).then_some(source)
}

fn propagate(compiled: &Compiled, domains: &mut [Vec<usize>]) -> bool {
    if domains.iter().any(Vec::is_empty) {
        return false;
    }
    let mut queue = VecDeque::from((0..compiled.arcs.len()).collect::<Vec<_>>());
    let mut queued = vec![true; compiled.arcs.len()];
    while let Some(arc_index) = queue.pop_front() {
        queued[arc_index] = false;
        if let Some(changed) = revise(compiled, domains, compiled.arcs[arc_index]) {
            if domains[changed].is_empty() {
                return false;
            }
            for &neighbor_index in &compiled.incoming_arcs[changed] {
                if !queued[neighbor_index] {
                    queue.push_back(neighbor_index);
                    queued[neighbor_index] = true;
                }
            }
        }
    }
    true
}

fn search(
    compiled: &Compiled,
    domains: Vec<Vec<usize>>,
    budget: usize,
    states_examined: &mut usize,
) -> Result<Option<Vec<Vec<usize>>>, SolveError> {
    let mut pending = vec![domains];
    while let Some(mut domains) = pending.pop() {
        if *states_examined == budget {
            return Err(SolveError::Exhausted {
                budget,
                states_examined: *states_examined,
            });
        }
        *states_examined += 1;
        if !propagate(compiled, &mut domains) {
            continue;
        }
        let choice = domains
            .iter()
            .enumerate()
            .filter(|(_, values)| values.len() > 1)
            .min_by_key(|(index, values)| (values.len(), *index))
            .map(|(index, _)| index);
        let Some(index) = choice else {
            return Ok(Some(domains));
        };
        for &value in domains[index].iter().rev() {
            let mut next = domains.clone();
            next[index] = vec![value];
            pending.push(next);
        }
    }
    Ok(None)
}

/// Домены после unary и дуговой согласованности, без перебора.
/// Сохранённое значение может ещё не входить в глобальное решение CSP.
pub fn viable_domains(problem: &Problem) -> Result<BTreeMap<String, Vec<String>>, SolveError> {
    let (compiled, mut domains) = compile(problem)?;
    if !propagate(&compiled, &mut domains) {
        return Err(SolveError::Unsat);
    }
    Ok(compiled
        .ids
        .iter()
        .enumerate()
        .map(|(index, id)| {
            (
                id.clone(),
                domains[index]
                    .iter()
                    .map(|&value| compiled.values[index][value].clone())
                    .collect(),
            )
        })
        .collect())
}

pub fn solve(problem: &Problem) -> Result<Solution, SolveError> {
    let (compiled, domains) = compile(problem)?;
    let mut states_examined = 0;
    let Some(domains) = search(&compiled, domains, problem.max_states, &mut states_examined)?
    else {
        return Err(SolveError::Unsat);
    };
    let assignment = compiled
        .ids
        .into_iter()
        .zip(domains)
        .enumerate()
        .map(|(index, (id, domain))| (id, compiled.values[index][domain[0]].clone()))
        .collect();
    Ok(Solution {
        assignment,
        states_examined,
    })
}

/// Диагностический проход после `Unsat`. Не объявляет конфликт при исчерпании
/// бюджета: для такого случая возвращает `Exhausted`.
pub fn first_unsat_component(problem: &Problem) -> Result<Option<ConflictComponent>, SolveError> {
    let (compiled, _) = compile(problem)?;
    let ids = compiled.ids;
    let index_by_id = ids
        .iter()
        .enumerate()
        .map(|(index, id)| (id.as_str(), index))
        .collect::<BTreeMap<_, _>>();
    let mut neighbors = vec![Vec::new(); ids.len()];
    for constraint in &problem.binary {
        let left = index_by_id[constraint.left.as_str()];
        let right = index_by_id[constraint.right.as_str()];
        neighbors[left].push(right);
        neighbors[right].push(left);
    }
    let mut visited = vec![false; ids.len()];
    let mut first_exhausted = None;
    for first in 0..ids.len() {
        if visited[first] {
            continue;
        }
        let mut pending = vec![first];
        visited[first] = true;
        let mut members = Vec::new();
        while let Some(index) = pending.pop() {
            members.push(index);
            for &neighbor in &neighbors[index] {
                if !visited[neighbor] {
                    visited[neighbor] = true;
                    pending.push(neighbor);
                }
            }
        }
        members.sort_unstable();
        let member_ids = members
            .iter()
            .map(|&index| ids[index].clone())
            .collect::<Vec<_>>();
        let member_set = member_ids
            .iter()
            .map(String::as_str)
            .collect::<BTreeSet<_>>();
        let unary_constraint_indices = problem
            .unary
            .iter()
            .enumerate()
            .filter_map(|(index, constraint)| {
                member_set
                    .contains(constraint.variable.as_str())
                    .then_some(index)
            })
            .collect::<Vec<_>>();
        let binary_constraint_indices = problem
            .binary
            .iter()
            .enumerate()
            .filter_map(|(index, constraint)| {
                (member_set.contains(constraint.left.as_str())
                    && member_set.contains(constraint.right.as_str()))
                .then_some(index)
            })
            .collect::<Vec<_>>();
        let subproblem = Problem {
            variables: problem
                .variables
                .iter()
                .filter(|variable| member_set.contains(variable.id.as_str()))
                .cloned()
                .collect(),
            unary: unary_constraint_indices
                .iter()
                .map(|&index| problem.unary[index].clone())
                .collect(),
            binary: binary_constraint_indices
                .iter()
                .map(|&index| problem.binary[index].clone())
                .collect(),
            max_states: problem.max_states,
        };
        match solve(&subproblem) {
            Err(SolveError::Unsat) => {
                return Ok(Some(ConflictComponent {
                    variables: member_ids,
                    unary_constraint_indices,
                    binary_constraint_indices,
                }))
            }
            Err(error @ SolveError::Exhausted { .. }) => {
                if first_exhausted.is_none() {
                    first_exhausted = Some(error);
                }
            }
            Err(error) => return Err(error),
            Ok(_) => {}
        }
    }
    if let Some(error) = first_exhausted {
        return Err(error);
    }
    Ok(None)
}

#[cfg(test)]
mod tests {
    use super::{
        first_unsat_component, solve, viable_domains, BinaryConstraint, ConflictComponent, Problem,
        SolveError, UnaryConstraint, Variable,
    };

    fn variable(id: &str, values: &[&str]) -> Variable {
        Variable {
            id: id.into(),
            values: values.iter().map(|value| (*value).into()).collect(),
        }
    }

    #[test]
    fn mirrored_tables_and_reordered_domains_choose_the_same_assignment() {
        let first = Problem {
            variables: vec![variable("z", &["b", "a"]), variable("a", &["b", "a"])],
            unary: vec![],
            binary: vec![BinaryConstraint {
                left: "a".into(),
                right: "z".into(),
                allowed: vec![("a".into(), "b".into()), ("b".into(), "a".into())],
            }],
            max_states: 100,
        };
        let mirrored = Problem {
            variables: vec![variable("a", &["a", "b"]), variable("z", &["a", "b"])],
            unary: vec![],
            binary: vec![BinaryConstraint {
                left: "z".into(),
                right: "a".into(),
                allowed: vec![("a".into(), "b".into()), ("b".into(), "a".into())],
            }],
            max_states: 100,
        };
        let left = solve(&first).unwrap();
        let right = solve(&mirrored).unwrap();
        assert_eq!(left.assignment, right.assignment);
        assert_eq!(left.assignment["a"], "a");
        assert_eq!(left.assignment["z"], "b");
    }

    #[test]
    fn unary_filter_propagates_through_sparse_table() {
        let problem = Problem {
            variables: vec![
                variable("a", &["short", "full"]),
                variable("b", &["short", "full"]),
            ],
            unary: vec![UnaryConstraint {
                variable: "a".into(),
                allowed: vec!["short".into()],
            }],
            binary: vec![BinaryConstraint {
                left: "a".into(),
                right: "b".into(),
                allowed: vec![
                    ("short".into(), "full".into()),
                    ("full".into(), "short".into()),
                ],
            }],
            max_states: 1,
        };
        let solution = solve(&problem).unwrap();
        assert_eq!(solution.assignment["b"], "full");
        assert_eq!(solution.states_examined, 1);
    }

    #[test]
    fn viable_domains_honor_forced_unary_without_searching_independent_options() {
        let problem = Problem {
            variables: vec![
                variable("forced", &["0", "1"]),
                variable("linked", &["0", "1"]),
                variable("independent", &["0", "1"]),
            ],
            unary: vec![UnaryConstraint {
                variable: "forced".into(),
                allowed: vec!["1".into()],
            }],
            binary: vec![BinaryConstraint {
                left: "forced".into(),
                right: "linked".into(),
                allowed: vec![("0".into(), "1".into()), ("1".into(), "0".into())],
            }],
            max_states: 1,
        };
        let domains = viable_domains(&problem).unwrap();
        assert_eq!(domains["forced"], vec!["1"]);
        assert_eq!(domains["linked"], vec!["0"]);
        assert_eq!(domains["independent"], vec!["0", "1"]);
        assert!(matches!(solve(&problem), Err(SolveError::Exhausted { .. })));
    }

    #[test]
    fn viable_domains_refuse_horizontal_contradiction() {
        let problem = Problem {
            variables: vec![variable("a", &["0"]), variable("b", &["0"])],
            unary: vec![],
            binary: vec![BinaryConstraint {
                left: "a".into(),
                right: "b".into(),
                allowed: vec![],
            }],
            max_states: 1,
        };
        assert_eq!(viable_domains(&problem), Err(SolveError::Unsat));
    }

    #[test]
    fn empty_compatibility_table_is_unsat() {
        let problem = Problem {
            variables: vec![variable("a", &["x"]), variable("b", &["y"])],
            unary: vec![],
            binary: vec![BinaryConstraint {
                left: "a".into(),
                right: "b".into(),
                allowed: vec![],
            }],
            max_states: 10,
        };
        assert_eq!(solve(&problem), Err(SolveError::Unsat));
    }

    #[test]
    fn finite_search_budget_reports_exhaustion_separately() {
        let problem = Problem {
            variables: vec![variable("a", &["0", "1"]), variable("b", &["0", "1"])],
            unary: vec![],
            binary: vec![],
            max_states: 1,
        };
        assert_eq!(
            solve(&problem),
            Err(SolveError::Exhausted {
                budget: 1,
                states_examined: 1
            })
        );
    }

    #[test]
    fn inconsistent_unary_and_binary_constraints_are_unsat() {
        let problem = Problem {
            variables: vec![variable("a", &["0", "1"]), variable("b", &["0", "1"])],
            unary: vec![
                UnaryConstraint {
                    variable: "a".into(),
                    allowed: vec!["0".into()],
                },
                UnaryConstraint {
                    variable: "b".into(),
                    allowed: vec!["1".into()],
                },
            ],
            binary: vec![BinaryConstraint {
                left: "a".into(),
                right: "b".into(),
                allowed: vec![("0".into(), "0".into()), ("1".into(), "1".into())],
            }],
            max_states: 10,
        };
        assert_eq!(solve(&problem), Err(SolveError::Unsat));
    }

    #[test]
    fn odd_cycle_requires_search_before_proving_unsat() {
        let unequal = |left: &str, right: &str| BinaryConstraint {
            left: left.into(),
            right: right.into(),
            allowed: vec![("0".into(), "1".into()), ("1".into(), "0".into())],
        };
        let mut problem = Problem {
            variables: vec![
                variable("c", &["1", "0"]),
                variable("a", &["1", "0"]),
                variable("b", &["1", "0"]),
            ],
            unary: vec![],
            binary: vec![unequal("a", "b"), unequal("b", "c"), unequal("c", "a")],
            max_states: 10,
        };
        assert_eq!(solve(&problem), Err(SolveError::Unsat));
        problem.max_states = 2;
        assert_eq!(
            solve(&problem),
            Err(SolveError::Exhausted {
                budget: 2,
                states_examined: 2
            })
        );
    }

    #[test]
    fn thousand_link_chain_propagates_in_one_state_independent_of_input_order() {
        let id = |index: usize| format!("node-{index:04}");
        let mut problem = Problem {
            variables: (0..1_000)
                .rev()
                .map(|index| Variable {
                    id: id(index),
                    values: vec!["1".into(), "0".into()],
                })
                .collect(),
            unary: vec![UnaryConstraint {
                variable: id(0),
                allowed: vec!["1".into()],
            }],
            binary: (0..999)
                .rev()
                .map(|index| BinaryConstraint {
                    left: id(index),
                    right: id(index + 1),
                    allowed: vec![("1".into(), "1".into()), ("0".into(), "0".into())],
                })
                .collect(),
            max_states: 1,
        };
        let first = solve(&problem).unwrap();
        problem.variables.reverse();
        problem.binary.reverse();
        let second = solve(&problem).unwrap();
        assert_eq!(first.assignment, second.assignment);
        assert_eq!(first.assignment.len(), 1_000);
        assert!(first.assignment.values().all(|value| value == "1"));
        assert_eq!(first.states_examined, 1);
    }

    #[test]
    fn thousand_independent_variables_use_iterative_search() {
        let problem = Problem {
            variables: (0..1_000)
                .map(|index| Variable {
                    id: format!("node-{index:04}"),
                    values: vec!["1".into(), "0".into()],
                })
                .collect(),
            unary: vec![],
            binary: vec![],
            max_states: 1_001,
        };
        let solution = solve(&problem).unwrap();
        assert_eq!(solution.assignment.len(), 1_000);
        assert!(solution.assignment.values().all(|value| value == "0"));
        assert_eq!(solution.states_examined, 1_001);
    }

    #[test]
    fn diagnostic_finds_only_the_first_proven_unsat_component() {
        let problem = Problem {
            variables: vec![
                variable("y", &["0"]),
                variable("a", &["0"]),
                variable("x", &["0"]),
                variable("b", &["0"]),
            ],
            unary: vec![],
            binary: vec![
                BinaryConstraint {
                    left: "x".into(),
                    right: "y".into(),
                    allowed: vec![],
                },
                BinaryConstraint {
                    left: "a".into(),
                    right: "b".into(),
                    allowed: vec![("0".into(), "0".into())],
                },
            ],
            max_states: 10,
        };
        assert_eq!(
            first_unsat_component(&problem),
            Ok(Some(ConflictComponent {
                variables: vec!["x".into(), "y".into()],
                unary_constraint_indices: vec![],
                binary_constraint_indices: vec![0],
            }))
        );
    }

    #[test]
    fn diagnostic_does_not_mislabel_exhaustion_as_conflict() {
        let problem = Problem {
            variables: vec![variable("a", &["0", "1"])],
            unary: vec![],
            binary: vec![],
            max_states: 1,
        };
        assert_eq!(
            first_unsat_component(&problem),
            Err(SolveError::Exhausted {
                budget: 1,
                states_examined: 1,
            })
        );
    }

    #[test]
    fn diagnostic_continues_after_exhausted_component_to_find_proven_unsat() {
        let problem = Problem {
            variables: vec![
                variable("a", &["0", "1"]),
                variable("x", &["0"]),
                variable("y", &["0"]),
            ],
            unary: vec![],
            binary: vec![BinaryConstraint {
                left: "x".into(),
                right: "y".into(),
                allowed: vec![],
            }],
            max_states: 1,
        };
        assert_eq!(
            first_unsat_component(&problem).unwrap().unwrap().variables,
            vec!["x", "y"]
        );
    }
}
