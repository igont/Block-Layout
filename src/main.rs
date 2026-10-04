use std::fs::{self, File, OpenOptions};
use std::io::{BufReader, BufWriter, Write};
use std::path::{Path, PathBuf};

use fb_layout::api::LayoutRequest;
use fb_layout::exchange::{self, ExchangeRequest};
use fb_layout::layout::Profile;

fn argument(name: &str) -> Result<PathBuf, String> {
    let mut args = std::env::args_os().skip(1);
    while let Some(key) = args.next() {
        let value = args
            .next()
            .ok_or_else(|| format!("missing value for {key:?}"))?;
        if key == name {
            return Ok(PathBuf::from(value));
        }
    }
    Err(format!("missing {name}; usage: fb-layout --request input.json --profile catalog.json --result result.json"))
}

fn optional_argument(name: &str) -> Option<PathBuf> {
    argument(name).ok()
}

fn read_json<T: serde::de::DeserializeOwned>(path: &Path) -> Result<T, String> {
    let file = File::open(path).map_err(|e| format!("{}: {e}", path.display()))?;
    serde_json::from_reader(BufReader::new(file)).map_err(|e| format!("{}: {e}", path.display()))
}

fn publish<T: serde::Serialize>(path: &Path, value: &T) -> Result<(), String> {
    let parent = path
        .parent()
        .ok_or_else(|| "result path has no parent".to_string())?;
    fs::create_dir_all(parent).map_err(|e| format!("{}: {e}", parent.display()))?;
    let mut temporary = path.as_os_str().to_os_string();
    temporary.push(".tmp");
    let temporary = PathBuf::from(temporary);
    let file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&temporary)
        .map_err(|e| format!("{}: {e}", temporary.display()))?;
    let mut writer = BufWriter::new(file);
    serde_json::to_writer(&mut writer, value).map_err(|e| e.to_string())?;
    writer.flush().map_err(|e| e.to_string())?;
    writer
        .into_inner()
        .map_err(|e| e.to_string())?
        .sync_all()
        .map_err(|e| e.to_string())?;
    fs::rename(&temporary, path).map_err(|e| format!("{}: {e}", path.display()))
}

fn run() -> Result<bool, String> {
    let request_path = argument("--request")?;
    let profile_path = argument("--profile")?;
    let result_path = argument("--result")?;
    if request_path == result_path {
        return Err("request and result must be different files".into());
    }
    let profile: Profile = read_json(&profile_path)?;
    let input: serde_json::Value = read_json(&request_path)?;
    let (request, standard) = if input.get("format").is_some() {
        let standard: ExchangeRequest = serde_json::from_value(input).map_err(|e| e.to_string())?;
        let converted = standard
            .to_layout_request()
            .and_then(|r| standard.validate_profile(&profile).map(|_| r));
        match converted {
            Ok(request) => (request, standard),
            Err(error) => {
                publish(&result_path, &exchange::failure_result(&standard, &[error]))?;
                return Ok(false);
            }
        }
    } else {
        // Явный локальный адаптер сохранённых снимков; публичный результат нейтрален.
        let request: LayoutRequest = serde_json::from_value(input).map_err(|e| e.to_string())?;
        let standard = exchange::from_layout_request(&request, &profile).map_err(|e| e.message)?;
        (request, standard)
    };
    let mut candidate = fb_layout::inspect_request(&request, &profile);
    if let Some(path) = optional_argument("--candidate") {
        publish(&path, &candidate)?;
    }
    if candidate.blocks.is_empty() {
        publish(
            &result_path,
            &exchange::failure_result(&standard, &candidate.diagnostics),
        )?;
        return Ok(false);
    }
    if optional_argument("--codes-only").as_deref() == Some(Path::new("true")) {
        let result = match fb_layout::code_result::export_codes(
            &standard,
            &request,
            &profile,
            &candidate.blocks,
            &candidate.diagnostics,
        ) {
            Ok(value) => value,
            Err(error) => {
                publish(&result_path, &exchange::failure_result(&standard, &[error]))?;
                return Ok(false);
            }
        };
        publish(&result_path, &result)?;
        println!(
            "blocks={} warnings={} layout_ms={} output=codes",
            candidate.blocks.len(),
            candidate.diagnostics.len(),
            candidate.elapsed_ms
        );
        return Ok(true);
    }
    let geometry_started = std::time::Instant::now();
    let materialized =
        match fb_layout::materialize::materialize(&request, &candidate.blocks, &profile) {
            Ok(v) => v,
            Err(e) => {
                candidate.diagnostics.push(e);
                publish(
                    &result_path,
                    &exchange::failure_result(&standard, &candidate.diagnostics),
                )?;
                return Ok(false);
            }
        };
    candidate.diagnostics.extend(materialized.diagnostics);
    let geometry_ms = geometry_started.elapsed().as_millis();
    if let Some(path) = optional_argument("--physical") {
        let mut view = serde_json::to_value(&materialized.physical).map_err(|e| e.to_string())?;
        view["warnings"] =
            serde_json::to_value(&candidate.diagnostics).map_err(|e| e.to_string())?;
        view["layout_elapsed_ms"] = serde_json::json!(candidate.elapsed_ms);
        view["geometry_elapsed_ms"] = serde_json::json!(geometry_ms);
        publish(&path, &view)?;
    }
    let result = exchange::success_result_with_warnings(
        &standard,
        materialized.exchange_blocks,
        fb_layout::effective_geometry::beam_adjustments(&request),
        &candidate.diagnostics,
    );
    publish(&result_path, &result)?;
    println!(
        "blocks={} warnings={} layout_ms={} geometry_ms={}",
        candidate.blocks.len(),
        candidate.diagnostics.len(),
        candidate.elapsed_ms,
        geometry_ms
    );
    Ok(true)
}

fn main() {
    match run() {
        Ok(true) => {}
        Ok(false) => std::process::exit(1),
        Err(error) => {
            eprintln!("fb-layout: {error}");
            std::process::exit(2);
        }
    }
}
