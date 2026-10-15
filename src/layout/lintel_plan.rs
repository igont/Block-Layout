//! Exact local selection of whole opening sets and their stock ownership.
use super::*;

struct Variant {
    removed: BTreeSet<String>,
    bridges: Vec<Block>,
}

struct Row {
    candidates: Vec<crate::constraints::LintelCandidate>,
    openings: Vec<usize>,
    layer: usize,
    layers: Vec<usize>,
    variants: [Option<Variant>; 2],
}

#[derive(Clone, PartialEq, Eq, PartialOrd, Ord)]
struct Score {
    total: usize,
    fairness: Vec<usize>,
    lower: Vec<usize>,
    unshifted: usize,
    spatial: Vec<u8>,
}

struct Search<'a> {
    rows: &'a [Row],
    indices: &'a [usize],
    opening_count: usize,
    packet_count: usize,
    best: Option<(Score, Vec<Option<usize>>)>,
}

impl Search<'_> {
    fn optimistic_score(
        &self,
        position: usize,
        offsets: &[Option<usize>],
        used: &BTreeSet<String>,
        claimed: &BTreeSet<(usize, usize)>,
        choices: &[Option<usize>],
    ) -> Score {
        let mut possible = claimed.clone();
        let mut spatial: Vec<_> = choices
            .iter()
            .map(|choice| match choice {
                Some(0) => 2,
                Some(_) => 1,
                None => 0,
            })
            .collect();
        for &index in &self.indices[position..] {
            let row = &self.rows[index];
            let claims: Vec<_> = row
                .openings
                .iter()
                .copied()
                .zip(row.layers.iter().copied())
                .collect();
            let mut preference = 0;
            if !claims.iter().any(|claim| claimed.contains(claim)) {
                for offset in 0..2 {
                    if row.openings.iter().any(|opening| offsets[*opening].is_some_and(|fixed| fixed != offset)) {
                        continue;
                    }
                    if row.variants[offset]
                        .as_ref()
                        .is_some_and(|variant| variant.removed.is_disjoint(used))
                    {
                        possible.extend(claims.iter().copied());
                        preference = preference.max(if offset == 0 { 2 } else { 1 });
                    }
                }
            }
            spatial.push(preference);
        }
        // This union deliberately ignores conflicts between future rows: each
        // completion is a subset, so every objective field is an upper bound.
        // Duplicated run representations contribute only one opening layer.
        let mut counts = vec![0; self.opening_count];
        let mut lower = vec![
            0;
            self.rows
                .iter()
                .flat_map(|r| r.layers.iter().copied())
                .max()
                .unwrap_or(0)
                + 1
        ];
        for &(opening, layer) in &possible {
            counts[opening] += 1;
            lower[layer] += 1;
        }
        counts.sort();
        Score {
            total: possible.len(),
            fairness: counts,
            lower,
            unshifted: self.packet_count
                - offsets.iter().filter(|offset| **offset == Some(1)).count(),
            spatial,
        }
    }

    fn visit(
        &mut self,
        position: usize,
        offsets: &mut [Option<usize>],
        used: &mut BTreeSet<String>,
        claimed: &mut BTreeSet<(usize, usize)>,
        choices: &mut Vec<Option<usize>>,
        total: usize,
    ) {
        let upper = self.optimistic_score(position, offsets, used, claimed, choices);
        if self.best.as_ref().is_some_and(|(score, _)| upper <= *score) {
            return;
        }
        if position == self.indices.len() {
            debug_assert_eq!(upper.total, total);
            self.best = Some((upper, choices.clone()));
            return;
        }
        let rows = self.rows;
        let row = &rows[self.indices[position]];
        let row_claims: Vec<_> = row
            .openings
            .iter()
            .copied()
            .zip(row.layers.iter().copied())
            .collect();
        for offset in 0..2 {
            if row.openings.iter().any(|opening| offsets[*opening].is_some_and(|fixed| fixed != offset)) {
                continue;
            }
            let Some(variant) = &row.variants[offset] else {
                continue;
            };
            if !variant.removed.is_disjoint(used) {
                continue;
            }
            // An opening crossing a T can have the same layer represented by
            // two runs. Those are alternatives, never two delivered layers.
            if row_claims.iter().any(|claim| claimed.contains(claim)) {
                continue;
            }
            let previous: Vec<_> = row.openings.iter().map(|opening| offsets[*opening]).collect();
            for &opening in &row.openings { offsets[opening] = Some(offset); }
            used.extend(variant.removed.iter().cloned());
            claimed.extend(row_claims.iter().copied());
            choices.push(Some(offset));
            self.visit(
                position + 1,
                offsets,
                used,
                claimed,
                choices,
                total + row.openings.len(),
            );
            choices.pop();
            for id in &variant.removed {
                used.remove(id);
            }
            for claim in &row_claims {
                claimed.remove(claim);
            }
            for (&opening, previous) in row.openings.iter().zip(previous) { offsets[opening] = previous; }
        }
        choices.push(None);
        self.visit(position + 1, offsets, used, claimed, choices, total);
        choices.pop();
    }
}

fn root(parents: &mut [usize], mut index: usize) -> usize {
    while parents[index] != index {
        parents[index] = parents[parents[index]];
        index = parents[index];
    }
    index
}

pub(super) fn merge(
    blocks: &mut Vec<Block>,
    topology: &Topology,
    constraints: &Constraints,
    profile: &Profile,
) -> BTreeSet<(i64, String)> {
    let mut completed = BTreeSet::new();
    let mut consoles = constraints.clone();
    consoles
        .lintel_candidates
        .retain(|c| constraints.console_axes.contains_key(&c.opening_id));
    completed.extend(merge_t_lintels_at_offset(
        blocks,
        &consoles,
        profile,
        0,
        &mut BTreeSet::new(),
        &mut false,
    ));

    let mut requested = constraints.clone();
    requested.lintel_candidates.retain(|c| {
        !constraints.console_axes.contains_key(&c.opening_id)
            && constraints.opening_axis(&c.opening_id).is_some()
    });
    let mut ids: Vec<_> = requested
        .lintel_candidates
        .iter()
        .map(|c| c.opening_id.clone())
        .collect();
    ids.sort();
    ids.dedup();
    ids.sort_by_key(|id| {
        let (a, b) = constraints.opening_axis(id).unwrap();
        (
            (a.x_mm.min(b.x_mm) * 100.).round() as i64,
            (a.y_mm.min(b.y_mm) * 100.).round() as i64,
            (a.x_mm.max(b.x_mm) * 100.).round() as i64,
            (a.y_mm.max(b.y_mm) * 100.).round() as i64,
        )
    });
    let mut first_courses = Vec::new();
    for id in &ids {
        let first = requested
            .lintel_candidates
            .iter()
            .filter(|c| &c.opening_id == id)
            .min_by_key(|c| c.course_index)
            .unwrap()
            .clone();
        first_courses.push(first.course_index);
        let (a, b) = constraints.opening_axis(id).unwrap();
        let count = if (b.x_mm - a.x_mm).hypot(b.y_mm - a.y_mm) <= 1000. {
            3
        } else {
            5
        };
        for layer in 0..count {
            let course = first.course_index + layer * 2;
            if !requested
                .lintel_candidates
                .iter()
                .any(|c| &c.opening_id == id && c.course_index == course)
            {
                let mut candidate = first.clone();
                candidate.course_index = course;
                requested.lintel_candidates.push(candidate);
            }
        }
    }
    if ids.is_empty() {
        return completed;
    }
    let opening_index: BTreeMap<_, _> = ids
        .iter()
        .enumerate()
        .map(|(i, id)| (id.clone(), i))
        .collect();
    let mut groups = Vec::new();
    let keys: BTreeSet<_> = requested
        .lintel_candidates
        .iter()
        .map(|c| (c.run_id.clone(), c.course_index))
        .collect();
    for (run, course) in keys {
        for group in lintel_groups(&requested, &run, course) {
            let mut group: Vec<_> = group.into_iter().cloned().collect();
            group.sort_by_key(|candidate| opening_index[&candidate.opening_id]);
            group.dedup_by(|a, b| a.opening_id == b.opening_id);
            if group.len() > 1 {
                groups.extend(group.iter().map(|candidate| vec![candidate.clone()]));
            }
            groups.push(group);
        }
    }
    let original_ids: BTreeSet<_> = blocks.iter().map(|b| b.id.clone()).collect();
    let mut rows = Vec::new();
    for candidates in groups {
        let openings: Vec<_> = candidates
            .iter()
            .map(|c| opening_index[&c.opening_id])
            .collect();
        let layers: Vec<_> = candidates
            .iter()
            .map(|c| ((c.course_index - first_courses[opening_index[&c.opening_id]]) / 2) as usize)
            .collect();
        let layer = *layers.iter().min().unwrap();
        let variants = std::array::from_fn(|offset| {
            if !candidates.iter().all(|c| {
                lintel_course_has_wall_height(topology, &requested, c, profile, offset as i64)
            }) {
                return None;
            }
            let mut subset = requested.clone();
            subset.lintel_candidates = candidates.clone();
            let mut trial = blocks.clone();
            let placed = merge_t_lintels_at_offset(
                &mut trial,
                &subset,
                profile,
                offset as i64,
                &mut BTreeSet::new(),
                &mut false,
            );
            if !candidates
                .iter()
                .all(|c| placed.contains(&(c.course_index, c.opening_id.clone())))
            {
                return None;
            }
            let retained: BTreeSet<_> = trial.iter().map(|b| b.id.clone()).collect();
            let removed = original_ids.difference(&retained).cloned().collect();
            let bridges = trial
                .into_iter()
                .filter(|b| !original_ids.contains(&b.id))
                .collect();
            Some(Variant { removed, bridges })
        });
        rows.push(Row {
            candidates,
            openings,
            layer,
            layers,
            variants,
        });
    }
    rows.sort_by_key(|row| (row.openings.iter().copied().min().unwrap(), row.layer));
    // Components share either a whole-set phase or a stock piece in any phase.
    let mut components: Vec<_> = (0..rows.len()).collect();
    let mut stock_owner = BTreeMap::new();
    let mut opening_owner = BTreeMap::new();
    for (index, row) in rows.iter().enumerate() {
        let mut neighbours = Vec::new();
        for &opening in &row.openings {
            if let Some(old) = opening_owner.insert(opening, index) {
                neighbours.push(old);
            }
        }
        for variant in row.variants.iter().flatten() {
            for id in &variant.removed {
                if let Some(old) = stock_owner.insert(id.clone(), index) {
                    neighbours.push(old);
                }
            }
        }
        for old in neighbours {
            let a = root(&mut components, index);
            let b = root(&mut components, old);
            components[b] = a;
        }
    }
    let mut local: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for index in 0..rows.len() {
        local
            .entry(root(&mut components, index))
            .or_default()
            .push(index);
    }
    let mut choices = vec![None; rows.len()];
    for indices in local.values() {
        let mut search = Search {
            rows: &rows,
            indices,
            opening_count: ids.len(),
            packet_count: ids.len(),
            best: None,
        };
        search.visit(
            0,
            &mut vec![None; ids.len()],
            &mut BTreeSet::new(),
            &mut BTreeSet::new(),
            &mut Vec::new(),
            0,
        );
        if let Some((_, best)) = search.best {
            for (&index, choice) in indices.iter().zip(best) {
                choices[index] = choice;
            }
        }
    }
    let mut offsets = vec![0; ids.len()];
    let mut removed = BTreeSet::new();
    let mut bridges = Vec::new();
    for (row, choice) in rows.iter().zip(&choices) {
        if let Some(offset) = choice {
            for &opening in &row.openings { offsets[opening] = *offset; }
            let variant = row.variants[*offset].as_ref().unwrap();
            removed.extend(variant.removed.iter().cloned());
            bridges.extend(variant.bridges.iter().cloned());
            completed.extend(
                row.candidates
                    .iter()
                    .map(|c| (c.course_index, c.opening_id.clone())),
            );
        }
    }
    blocks.retain(|b| !removed.contains(&b.id));
    blocks.extend(bridges);
    for (row, choice) in rows.iter().zip(&choices) {
        if choice.is_none() && row.layers.iter().any(|layer| *layer > 0) {
            for (candidate, layer) in row.candidates.iter().zip(&row.layers) {
                if *layer == 0 || completed.contains(&(candidate.course_index, candidate.opening_id.clone())) {
                    continue;
                }
                let mut subset = requested.clone();
                subset.lintel_candidates = vec![candidate.clone()];
                preserve_upper_lintel_rows(blocks, topology, &subset, profile,
                    offsets[opening_index[&candidate.opening_id]] as i64);
            }
        }
    }
    completed
}

#[cfg(test)]
mod tests {
    use super::*;

    fn row(packet: usize, opening: usize, layer: usize, stock: [Option<&[&str]>; 2]) -> Row {
        assert_eq!(packet, opening);
        Row {
            candidates: Vec::new(),
            openings: vec![opening],
            layer,
            layers: vec![layer],
            variants: stock.map(|ids| {
                ids.map(|ids| Variant {
                    removed: ids.iter().map(|id| (*id).to_owned()).collect(),
                    bridges: Vec::new(),
                })
            }),
        }
    }

    fn solve(rows: &[Row]) -> (Score, BTreeMap<(usize, usize), Option<usize>>) {
        // Opening indices represent the spatial order established by merge.
        let mut indices: Vec<_> = (0..rows.len()).collect();
        indices.sort_by_key(|i| (rows[*i].openings[0], rows[*i].layer));
        let count = rows.iter().flat_map(|r| r.openings.iter().copied()).max().unwrap() + 1;
        let mut search = Search {
            rows,
            indices: &indices,
            opening_count: count,
            packet_count: count,
            best: None,
        };
        search.visit(
            0,
            &mut vec![None; count],
            &mut BTreeSet::new(),
            &mut BTreeSet::new(),
            &mut Vec::new(),
            0,
        );
        let (score, choices) = search.best.unwrap();
        let mut selected = BTreeMap::new();
        for (index, choice) in indices.iter().zip(choices) {
            for (&opening, &layer) in rows[*index].openings.iter().zip(&rows[*index].layers) {
                let selected = selected.entry((opening, layer)).or_insert(None);
                if choice.is_some() { *selected = choice; }
            }
        }
        (score, selected)
    }

    #[test]
    fn moving_neighbour_recovers_more_rows_than_greedy_stock_ownership() {
        let mut rows = vec![
            row(0, 0, 0, [Some(&["shared"]), Some(&["alternate"])]),
            row(1, 1, 0, [Some(&["shared"]), None]),
            row(1, 1, 1, [Some(&["own-upper"]), None]),
        ];
        let (score, selected) = solve(&rows);
        assert_eq!(score.total, 3);
        assert_eq!(
            selected,
            BTreeMap::from([((0, 0), Some(1)), ((1, 0), Some(0)), ((1, 1), Some(0)),])
        );
        // Storage order cannot change the spatial tie-break or chosen phases.
        rows.reverse();
        let (reversed_score, reversed) = solve(&rows);
        assert!(score == reversed_score);
        assert_eq!(selected, reversed);
    }

    #[test]
    fn unavailable_upper_row_keeps_other_rows_in_one_shared_phase() {
        let rows = vec![
            row(
                0,
                0,
                0,
                [Some(&["lower-original"]), Some(&["lower-shifted"])],
            ),
            row(0, 0, 1, [None, Some(&["upper-shifted"])]),
            row(0, 0, 2, [None, None]),
        ];
        let (score, selected) = solve(&rows);
        assert_eq!(score.total, 2);
        assert_eq!(
            selected,
            BTreeMap::from([((0, 0), Some(1)), ((0, 1), Some(1)), ((0, 2), None),])
        );
    }

    #[test]
    fn equal_total_prefers_covering_both_openings() {
        let rows = vec![
            row(0, 0, 0, [Some(&["lower"]), None]),
            row(0, 0, 1, [Some(&["shared"]), None]),
            row(1, 1, 0, [Some(&["shared"]), None]),
        ];
        let (score, selected) = solve(&rows);
        assert_eq!(score.total, 2);
        assert_eq!(score.fairness, vec![1, 1]);
        assert_eq!(
            selected,
            BTreeMap::from([((0, 0), Some(0)), ((0, 1), None), ((1, 0), Some(0)),])
        );
    }

    #[test]
    fn one_layer_in_two_source_runs_is_counted_once_even_with_disjoint_stock() {
        let rows = vec![
            row(0, 0, 0, [Some(&["left-run"]), None]),
            row(0, 0, 0, [Some(&["right-run"]), None]),
            row(0, 0, 1, [Some(&["upper"]), None]),
        ];
        let (score, _) = solve(&rows);
        assert_eq!(score.total, 2);
        assert_eq!(score.fairness, vec![2]);
    }

    #[test]
    fn neighbouring_openings_can_use_independent_whole_set_phases() {
        let mut grouped = row(0, 0, 0,
            [Some(&["combined-original"]), Some(&["combined-shifted"])]);
        grouped.openings = vec![0, 1];
        grouped.layers = vec![0, 0];
        let rows = vec![grouped,
            row(0, 0, 0, [Some(&["own-lower"]), None]),
            row(0, 0, 1, [Some(&["own-upper"]), None]),
            row(1, 1, 0, [None, Some(&["neighbour-lower"])]),
            row(1, 1, 1, [None, Some(&["neighbour-upper"])]),
        ];
        let (score, selected) = solve(&rows);
        assert_eq!(score.total, 4);
        assert_eq!(selected, BTreeMap::from([
            ((0, 0), Some(0)), ((0, 1), Some(0)),
            ((1, 0), Some(1)), ((1, 1), Some(1)),
        ]));
    }

    #[test]
    fn unavailable_combined_row_keeps_the_valid_single_opening() {
        let mut grouped = row(0, 0, 0, [None, None]);
        grouped.openings = vec![0, 1];
        grouped.layers = vec![0, 0];
        let rows = vec![grouped,
            row(0, 0, 0, [Some(&["valid-neighbour"]), None]),
            row(1, 1, 0, [None, None]),
        ];
        let (score, selected) = solve(&rows);
        assert_eq!(score.total, 1);
        assert_eq!(selected[&(0, 0)], Some(0));
        assert_eq!(selected[&(1, 0)], None);
    }
}
