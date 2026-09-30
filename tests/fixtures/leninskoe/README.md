# Снимки текущей модели Ленинского

Три проекта (`dom`, `banya`, `garazh`) получены из `variant-0/.sup/model/current.sqlite` текущих каталогов «Дом», «Баня», «Гараж». `input.json.gz` содержит только входы раскладки: `wall_volumes` с `purposeType=1`, все `opening_volumes` и `beams`. `observed.json.gz` содержит прежние `observed_blocks` отдельно; их нельзя подавать движку как исходные данные.

В обоих файлах одного проекта совпадают `schema_version=1`, `project_name`, `project_id` (`projectKey` из `.sup/project.tsv`), `snapshot_hash` и `z0_mm`. `metadata` хранит SQLite `user_version`, `model_id`, `export_id`, а для каждой исходной секции — `row_count`, `selected_count`, `content_hash` и остальные поля `current_section`. `snapshot_hash` — SHA-256 от версии, метаданных секций и исходных CBOR-строк с длинами и порядком; он одинаков у входа и наблюдения. Десятичные числа декодируются в JSON как точные числовые литералы.

Экспорт читает SQLite через URI `mode=ro` в одной транзакции и пишет детерминированный gzip (`mtime=0`). Он проверяет непрерывность `ordinal`, число строк каждой секции и обратное чтение JSON. Для сравнения сохранённых fixtures с актуальными базами:

```powershell
python -m pip install --user cbor2 simplejson
python tools/export_sup_snapshots.py --verify
```

Для пересоздания без `--verify` запустите тот же скрипт. При изменившемся содержимом текущих баз проверка закономерно не пройдёт: это сигнал сверить новый экспорт с прежним, а не обновлять эталон автоматически.
