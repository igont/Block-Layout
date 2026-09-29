[CmdletBinding()]
param(
    [Parameter()]
    [string] $SourceRoot = 'C:\ForestBrick',

    [Parameter()]
    [string] $Destination = (Join-Path $PSScriptRoot '..\tests\fixtures\characterization\first-10-courses')
)

$ErrorActionPreference = 'Stop'
$utf8NoBom = [System.Text.UTF8Encoding]::new($false)
$courseHeightM = 0.063
$CourseCount = 10
$gridStepMm = 640
$gridOffsetMm = 320

function Write-Utf8File {
    param(
        [Parameter(Mandatory)]
        [string] $Path,

        [Parameter(Mandatory)]
        [AllowEmptyString()]
        [string] $Content
    )

    $parent = Split-Path -Parent $Path
    if ($parent) {
        [System.IO.Directory]::CreateDirectory($parent) | Out-Null
    }
    [System.IO.File]::WriteAllText($Path, $Content, $utf8NoBom)
}

function Copy-ExactFile {
    param(
        [Parameter(Mandatory)]
        [string] $Source,

        [Parameter(Mandatory)]
        [string] $Target
    )

    $parent = Split-Path -Parent $Target
    [System.IO.Directory]::CreateDirectory($parent) | Out-Null
    [System.IO.File]::WriteAllBytes($Target, [System.IO.File]::ReadAllBytes($Source))
}

function Get-Sha256 {
    param([Parameter(Mandatory)][string] $Path)
    return (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
}

function Get-ConfigValue {
    param(
        [Parameter(Mandatory)][AllowEmptyString()][string[]] $Lines,
        [Parameter(Mandatory)][string] $Key
    )

    $pattern = '^\s*' + [regex]::Escape($Key) + '\s*\|\s*(.*?)\s*$'
    foreach ($line in $Lines) {
        if ($line -match $pattern) {
            return $Matches[1]
        }
    }
    return $null
}

$sourceRootPath = [System.IO.Path]::GetFullPath($SourceRoot)
$destinationPath = [System.IO.Path]::GetFullPath($Destination)
$layersPath = Join-Path $sourceRootPath 'layers'
$openingsPath = Join-Path $sourceRootPath 'openings.json'
$blocksPath = Join-Path $sourceRootPath 'blocks.json'
$configPath = Join-Path $sourceRootPath 'config JAR.txt'
$logPath = Join-Path $sourceRootPath 'log JAR.txt'

foreach ($required in @($layersPath, $openingsPath, $blocksPath, $configPath, $logPath)) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "Не найден обязательный исходный артефакт: $required"
    }
}

$layerFiles = @(Get-ChildItem -LiteralPath $layersPath -Filter 'layer *.json' -File | ForEach-Object {
    $match = [regex]::Match($_.BaseName, '-?\d+')
    if (-not $match.Success) {
        throw "Не удалось определить отметку слоя из имени: $($_.Name)"
    }
    [pscustomobject]@{
        File = $_
        LevelMm = [int] $match.Value
    }
} | Sort-Object LevelMm)

if ($layerFiles.Count -lt 2) {
    throw 'Для определения диапазона исходной геометрии нужны минимум два файла слоя.'
}

$sourceLayer = $layerFiles[0]
$nextLayer = $layerFiles[1]
$sourceLayerJson = Get-Content -LiteralPath $sourceLayer.File.FullName -Raw -Encoding UTF8 | ConvertFrom-Json
$openings = @(Get-Content -LiteralPath $openingsPath -Raw -Encoding UTF8 | ConvertFrom-Json)
$blocks = @(Get-Content -LiteralPath $blocksPath -Raw -Encoding UTF8 | ConvertFrom-Json)
$configLines = @(Get-Content -LiteralPath $configPath -Encoding UTF8)

$firstZ = $sourceLayer.LevelMm / 1000.0
$firstLegacyFloor = [int] [math]::Round($firstZ / $courseHeightM) + 4
$lastLegacyFloor = $firstLegacyFloor + $CourseCount - 1
$lastZMm = $sourceLayer.LevelMm + (($CourseCount - 1) * [int] [math]::Round($courseHeightM * 1000))

if ($lastZMm -ge $nextLayer.LevelMm) {
    throw "Первые $CourseCount венцов пересекают смену входной геометрии на отметке $($nextLayer.LevelMm) мм. Экспортёр пока фиксирует один исходный слой."
}

$courses = @()
for ($index = 0; $index -lt $CourseCount; $index++) {
    $zMm = $sourceLayer.LevelMm + ($index * 63)
    $legacyFloor = $firstLegacyFloor + $index
    $courseBlocks = @($blocks | Where-Object { [int] $_.floor -eq $legacyFloor })

    $courses += [ordered]@{
        course_index = $index
        z_mm = $zMm
        legacy_floor = $legacyFloor
        block_count = $courseBlocks.Count
        blocks = $courseBlocks
    }
}

$selectedBlocks = @($blocks | Where-Object {
    [int] $_.floor -ge $firstLegacyFloor -and [int] $_.floor -le $lastLegacyFloor
})

$blocksInfo = Get-Item -LiteralPath $blocksPath
$logInfo = Get-Item -LiteralPath $logPath
$capturedAtLocal = $blocksInfo.LastWriteTime.ToString('dd.MM.yyyy HH:mm')
$units = [ordered]@{
    coordinates = 'm'
    lengths = 'm'
    angles = 'rad'
    explicit_levels = 'mm'
}
$constants = [ordered]@{
    course_height_mm = 63
    standard_grid_step_mm = $gridStepMm
    standard_grid_offset_mm = $gridOffsetMm
    full_block_length_mm = 640
    half_block_length_mm = 320
    opening_side_gap_mm = [int] (Get-ConfigValue -Lines $configLines -Key 'Зазор по бокам от окон')
}
$selection = [ordered]@{
    first_course_index = 0
    course_count = $CourseCount
    start_z_mm = $sourceLayer.LevelMm
    end_z_mm = $lastZMm
    legacy_floor_from = $firstLegacyFloor
    legacy_floor_to = $lastLegacyFloor
    source_geometry_active_from_z_mm = $sourceLayer.LevelMm
    source_geometry_active_until_z_mm = $nextLayer.LevelMm
}
$provenance = [ordered]@{
    captured_at_local = $capturedAtLocal
    source_location_note = 'Исходный каталог не является частью переносимого контракта.'
    source_layer = [ordered]@{
        file = $sourceLayer.File.Name
        sha256 = Get-Sha256 $sourceLayer.File.FullName
        modified_at_local = $sourceLayer.File.LastWriteTime.ToString('dd.MM.yyyy HH:mm')
    }
    source_openings = [ordered]@{
        file = 'openings.json'
        sha256 = Get-Sha256 $openingsPath
        modified_at_local = (Get-Item -LiteralPath $openingsPath).LastWriteTime.ToString('dd.MM.yyyy HH:mm')
    }
    source_config = [ordered]@{
        file = 'config JAR.txt'
        sha256 = Get-Sha256 $configPath
        modified_at_local = (Get-Item -LiteralPath $configPath).LastWriteTime.ToString('dd.MM.yyyy HH:mm')
    }
    source_blocks = [ordered]@{
        file = 'blocks.json'
        sha256 = Get-Sha256 $blocksPath
        modified_at_local = $blocksInfo.LastWriteTime.ToString('dd.MM.yyyy HH:mm')
    }
    source_log = [ordered]@{
        file = 'log JAR.txt'
        sha256 = Get-Sha256 $logPath
        modified_at_local = $logInfo.LastWriteTime.ToString('dd.MM.yyyy HH:mm')
    }
    producer_binary = 'not-pinned'
    producer_note = 'Текущий ForestBrick.jar новее blocks.json, поэтому он не объявлен точным бинарным производителем этого результата.'
}
$blockFields = @(
    'posX',
    'posY',
    'angleRad',
    'floor',
    'switchToX',
    'length',
    'isDobor',
    'removePipesStart',
    'removePipesEnd',
    'uteplitAdd',
    'isBridge',
    'bridgeCode1',
    'bridgeCode2',
    'bridgeNominalLength',
    'sourceVolumeGuid',
    'sourceVolumeGuids',
    'cuts'
)
$comparison = [ordered]@{
    collection_semantics = 'unordered-multiset'
    linear_quantum_m = 0.000001
    angle_quantum_rad = 0.000000001
    source_volume_guids_semantics = 'unordered-set'
    cuts_semantics = 'unordered-multiset'
    compare_all_block_fields = $true
    block_fields = $blockFields
    cut_fields = @('cut', 'x', 'y')
}
$layoutRequest = [ordered]@{
    schema_version = '1.0'
    fixture_id = 'forestbrick-first-10-courses'
    units = $units
    constants = $constants
    selection = $selection
    source_geometry = [ordered]@{
        active_from_z_mm = $sourceLayer.LevelMm
        active_until_z_mm = $nextLayer.LevelMm
        line_count = @($sourceLayerJson.lines).Count
        lines = @($sourceLayerJson.lines)
    }
    opening_count = $openings.Count
    openings = $openings
}
$layoutResult = [ordered]@{
    schema_version = '1.0'
    fixture_id = 'forestbrick-first-10-courses'
    result_kind = 'legacy-java-observation'
    oracle_status = 'comparison-only-not-accepted-as-correct'
    comparison = $comparison
    total_block_count = $selectedBlocks.Count
    courses = $courses
}

$nonRegularLines = @()
for ($lineIndex = 0; $lineIndex -lt @($sourceLayerJson.lines).Count; $lineIndex++) {
    $line = $sourceLayerJson.lines[$lineIndex]
    $dxMm = ([double] $line.posBX - [double] $line.posAX) * 1000
    $dyMm = ([double] $line.posBY - [double] $line.posAY) * 1000
    $lengthMm = [int] [math]::Round([math]::Sqrt(($dxMm * $dxMm) + ($dyMm * $dyMm)))
    $remainderMm = $lengthMm % $gridStepMm
    if ($remainderMm -ne 0) {
        $nonRegularLines += [ordered]@{
            line_index = $lineIndex
            source_volume_guid = $line.sourceVolumeGuid
            length_mm = $lengthMm
            remainder_mm = $remainderMm
        }
    }
}

$assertions = [ordered]@{
    schema_version = '1.0'
    fixture_id = 'forestbrick-first-10-courses'
    integrity = [ordered]@{
        source_line_count = @($sourceLayerJson.lines).Count
        opening_count = $openings.Count
        course_count = $CourseCount
        course_height_mm = 63
        course_z_mm = @($courses | ForEach-Object { $_.z_mm })
    }
    characterization = [ordered]@{
        oracle_status = 'comparison-only-not-accepted-as-correct'
        total_block_count = $selectedBlocks.Count
        blocks_per_course = @($courses | ForEach-Object { $_.block_count })
    }
    normative = [ordered]@{
        legacy_block_output_assertions = @()
        regular_640_strategy = [ordered]@{
            expected_outcome = 'unsupported-input'
            reason_code = 'wall-length-not-divisible-by-grid-step'
            non_regular_source_lines = $nonRegularLines
        }
    }
}

$manifest = [ordered]@{
    schema_version = '1.0'
    fixture_id = 'forestbrick-first-10-courses'
    fixture_kind = 'characterization'
    description = 'Первые десять венцов сложной модели для переноса на новый движок раскладки.'
    provenance = $provenance
    files = [ordered]@{
        request = 'input/layout-request.v1.json'
        observed_result = 'observed/layout-result.v1.json'
        assertions = 'contract/assertions.v1.json'
        raw_layer = "raw/input/layer_$($sourceLayer.LevelMm).json"
        raw_openings = 'raw/input/openings.json'
        raw_config = 'raw/input/config_JAR.txt'
        checksums = 'SHA256SUMS.txt'
    }
    test_contract = [ordered]@{
        request_is_normative_input = $true
        observed_result_is_normative_output = $false
        observed_result_use = 'characterization-diff-only'
        comparison = $comparison
        normative_assertions_file = 'contract/assertions.v1.json'
    }
}

$rawInputPath = Join-Path $destinationPath 'raw\input'
$observedPath = Join-Path $destinationPath 'observed'
[System.IO.Directory]::CreateDirectory($rawInputPath) | Out-Null
[System.IO.Directory]::CreateDirectory($observedPath) | Out-Null
[System.IO.Directory]::CreateDirectory((Join-Path $destinationPath 'input')) | Out-Null
[System.IO.Directory]::CreateDirectory((Join-Path $destinationPath 'contract')) | Out-Null

foreach ($obsoleteRelativePath in @('fixture.v1.json', "observed/blocks.courses-$firstLegacyFloor-$lastLegacyFloor.json")) {
    $obsoletePath = Join-Path $destinationPath $obsoleteRelativePath
    if (Test-Path -LiteralPath $obsoletePath -PathType Leaf) {
        Remove-Item -LiteralPath $obsoletePath -Force
    }
}

Copy-ExactFile -Source $sourceLayer.File.FullName -Target (Join-Path $rawInputPath "layer_$($sourceLayer.LevelMm).json")
Copy-ExactFile -Source $openingsPath -Target (Join-Path $rawInputPath 'openings.json')
Copy-ExactFile -Source $configPath -Target (Join-Path $rawInputPath 'config_JAR.txt')

$layoutRequestJson = $layoutRequest | ConvertTo-Json -Depth 40
Write-Utf8File -Path (Join-Path $destinationPath 'input\layout-request.v1.json') -Content ($layoutRequestJson + "`n")

$layoutResultJson = $layoutResult | ConvertTo-Json -Depth 40
Write-Utf8File -Path (Join-Path $destinationPath 'observed\layout-result.v1.json') -Content ($layoutResultJson + "`n")

$assertionsJson = $assertions | ConvertTo-Json -Depth 20
Write-Utf8File -Path (Join-Path $destinationPath 'contract\assertions.v1.json') -Content ($assertionsJson + "`n")

$manifestJson = $manifest | ConvertTo-Json -Depth 20
Write-Utf8File -Path (Join-Path $destinationPath 'manifest.v1.json') -Content ($manifestJson + "`n")

$courseRows = $courses | ForEach-Object {
    "| $($_.course_index) | $($_.z_mm) | $($_.legacy_floor) | $($_.block_count) |"
}
$readme = @"
# Первые 10 венцов сложной модели

Это характеристический тестовый набор для будущего движка раскладки на Rust.

## Что зафиксировано

- фактический входной слой, действующий от $($sourceLayer.LevelMm) до $($nextLayer.LevelMm) мм;
- полный вход `openings.json`, потому что фильтрация проёмов является частью контракта движка;
- первые $CourseCount сгенерированных венцов от $($sourceLayer.LevelMm) до $lastZMm мм;
- наблюдавшийся результат старого Java-движка: $($selectedBlocks.Count) блоков.

**observed_output** не является утверждением, что раскладка правильная. Он нужен для сравнения, поиска осознанных отличий и защиты уже понятного поведения. Нормативные проверки сетки, соединений и рустов должны храниться отдельно от этой характеристики.

## Как использовать в новых тестах

1. Загрузить **manifest.v1.json**.
2. Передать **input/layout-request.v1.json** в адаптер или ядро.
3. Для характеристического сравнения загрузить **observed/layout-result.v1.json**.
4. Сравнивать блоки как неупорядоченное мультимножество по правилам из манифеста, а не по порядку JSON-массива.
5. Нормативные утверждения брать только из **contract/assertions.v1.json**. В нём намеренно нет утверждений о правильности старых блоков.

## Основные файлы

- **manifest.v1.json** - точка входа и машинный тестовый контракт;
- **input/layout-request.v1.json** - самодостаточный запрос для первых десяти венцов;
- **observed/layout-result.v1.json** - сгруппированный наблюдаемый результат старого Java-движка;
- **contract/assertions.v1.json** - отдельно зафиксированные нормативные и характеристические утверждения;
- **raw/input** - исходные файлы без преобразования;
- **SHA256SUMS.txt** - контрольные суммы всех файлов пакета.

## Состав венцов

| Индекс | Отметка, мм | Старое поле floor | Блоков |
|---:|---:|---:|---:|
$($courseRows -join "`n")

## Ограничение происхождения

**blocks.json** и **log JAR.txt** относятся к одному запуску от $($blocksInfo.LastWriteTime.ToString('dd.MM.yyyy HH:mm')). Текущий **ForestBrick.jar** был изменён позднее, поэтому точный бинарный производитель результата не заявлен.

## Проверка

Из корня старого проекта:

    .\tools\Test-RustRewriteFixture.ps1
"@
Write-Utf8File -Path (Join-Path $destinationPath 'README.md') -Content ($readme.TrimEnd() + "`n")

$filesToHash = @(Get-ChildItem -LiteralPath $destinationPath -File -Recurse | Where-Object { $_.Name -ne 'SHA256SUMS.txt' } | Sort-Object FullName)
$sumLines = foreach ($file in $filesToHash) {
    $relative = [System.IO.Path]::GetRelativePath($destinationPath, $file.FullName).Replace('\', '/')
    "$(Get-Sha256 $file.FullName)  $relative"
}
Write-Utf8File -Path (Join-Path $destinationPath 'SHA256SUMS.txt') -Content (($sumLines -join "`n") + "`n")

[pscustomobject]@{
    Destination = $destinationPath
    CourseCount = $CourseCount
    BlockCount = $selectedBlocks.Count
    OpeningCount = $openings.Count
    SourceLineCount = @($sourceLayerJson.lines).Count
}
