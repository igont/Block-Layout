[CmdletBinding()]
param(
    [Parameter()]
    [string] $FixtureRoot = (Join-Path $PSScriptRoot '..\tests\fixtures\characterization\first-10-courses')
)

$ErrorActionPreference = 'Stop'

function Read-JsonFile {
    param([Parameter(Mandatory)][string] $Path)

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw "Не найден JSON-файл: $Path"
    }
    return Get-Content -LiteralPath $Path -Raw -Encoding UTF8 | ConvertFrom-Json
}

function Assert-Equal {
    param(
        [Parameter(Mandatory)][AllowNull()] $Actual,
        [Parameter(Mandatory)][AllowNull()] $Expected,
        [Parameter(Mandatory)][string] $Message
    )

    if ($Actual -cne $Expected) {
        throw "$Message Ожидалось: '$Expected'; получено: '$Actual'."
    }
}

function Assert-JsonEqual {
    param(
        [Parameter(Mandatory)][AllowEmptyCollection()] $Actual,
        [Parameter(Mandatory)][AllowEmptyCollection()] $Expected,
        [Parameter(Mandatory)][string] $Message
    )

    $actualJson = ConvertTo-Json -InputObject $Actual -Depth 50 -Compress
    $expectedJson = ConvertTo-Json -InputObject $Expected -Depth 50 -Compress
    Assert-Equal -Actual $actualJson -Expected $expectedJson -Message $Message
}

function Resolve-FixtureFile {
    param(
        [Parameter(Mandatory)][string] $Root,
        [Parameter(Mandatory)][string] $RelativePath
    )

    $path = Join-Path $Root ($RelativePath.Replace('/', '\'))
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "Манифест ссылается на отсутствующий файл: $RelativePath"
    }
    return $path
}

$root = [System.IO.Path]::GetFullPath($FixtureRoot)
$manifestPath = Join-Path $root 'manifest.v1.json'
$manifest = Read-JsonFile $manifestPath

Assert-Equal -Actual $manifest.schema_version -Expected '1.0' -Message 'Неожиданная версия манифеста.'
Assert-Equal -Actual $manifest.fixture_id -Expected 'forestbrick-first-10-courses' -Message 'Неожиданный идентификатор фикстуры.'
Assert-Equal -Actual $manifest.fixture_kind -Expected 'characterization' -Message 'Фикстура должна оставаться характеристической.'
Assert-Equal -Actual $manifest.test_contract.observed_result_is_normative_output -Expected $false -Message 'Старый Java-выход нельзя объявлять нормативным.'
Assert-Equal -Actual $manifest.test_contract.comparison.collection_semantics -Expected 'unordered-multiset' -Message 'Порядок блоков не должен участвовать в сравнении.'

$checksumsPath = Resolve-FixtureFile -Root $root -RelativePath $manifest.files.checksums
$listedFiles = @{}
foreach ($line in Get-Content -LiteralPath $checksumsPath -Encoding UTF8) {
    if ($line -notmatch '^([0-9a-f]{64})  (.+)$') {
        throw "Некорректная строка SHA256SUMS.txt: $line"
    }

    $relativePath = $Matches[2]
    if ($listedFiles.ContainsKey($relativePath)) {
        throw "Файл дважды указан в SHA256SUMS.txt: $relativePath"
    }

    $filePath = Resolve-FixtureFile -Root $root -RelativePath $relativePath
    $actualHash = (Get-FileHash -LiteralPath $filePath -Algorithm SHA256).Hash.ToLowerInvariant()
    Assert-Equal -Actual $actualHash -Expected $Matches[1] -Message "Нарушена контрольная сумма файла $relativePath."
    $listedFiles[$relativePath] = $true
}

$actualFiles = @(Get-ChildItem -LiteralPath $root -Recurse -File |
    Where-Object { $_.FullName -ne $checksumsPath } |
    ForEach-Object { [System.IO.Path]::GetRelativePath($root, $_.FullName).Replace('\', '/') } |
    Sort-Object)
$listedPaths = @($listedFiles.Keys | Sort-Object)
Assert-JsonEqual -Actual $listedPaths -Expected $actualFiles -Message 'SHA256SUMS.txt должен перечислять каждый файл пакета ровно один раз.'

$requestPath = Resolve-FixtureFile -Root $root -RelativePath $manifest.files.request
$resultPath = Resolve-FixtureFile -Root $root -RelativePath $manifest.files.observed_result
$assertionsPath = Resolve-FixtureFile -Root $root -RelativePath $manifest.files.assertions
$rawLayerPath = Resolve-FixtureFile -Root $root -RelativePath $manifest.files.raw_layer
$rawOpeningsPath = Resolve-FixtureFile -Root $root -RelativePath $manifest.files.raw_openings
$rawConfigPath = Resolve-FixtureFile -Root $root -RelativePath $manifest.files.raw_config

$request = Read-JsonFile $requestPath
$result = Read-JsonFile $resultPath
$assertions = Read-JsonFile $assertionsPath
$rawLayer = Read-JsonFile $rawLayerPath
$rawOpenings = @(Read-JsonFile $rawOpeningsPath)

Assert-Equal -Actual (Get-FileHash -LiteralPath $rawLayerPath -Algorithm SHA256).Hash.ToLowerInvariant() -Expected $manifest.provenance.source_layer.sha256 -Message 'Сырой слой разошёлся с зафиксированным источником.'
Assert-Equal -Actual (Get-FileHash -LiteralPath $rawOpeningsPath -Algorithm SHA256).Hash.ToLowerInvariant() -Expected $manifest.provenance.source_openings.sha256 -Message 'Сырые проёмы разошлись с зафиксированным источником.'
Assert-Equal -Actual (Get-FileHash -LiteralPath $rawConfigPath -Algorithm SHA256).Hash.ToLowerInvariant() -Expected $manifest.provenance.source_config.sha256 -Message 'Сырая конфигурация разошлась с зафиксированным источником.'

foreach ($document in @($request, $result, $assertions)) {
    Assert-Equal -Actual $document.schema_version -Expected '1.0' -Message 'Неожиданная версия документа фикстуры.'
    Assert-Equal -Actual $document.fixture_id -Expected $manifest.fixture_id -Message 'Документы пакета относятся к разным фикстурам.'
}

Assert-JsonEqual -Actual @($request.source_geometry.lines) -Expected @($rawLayer.lines) -Message 'Нормализованная геометрия разошлась с сырым слоем.'
Assert-JsonEqual -Actual @($request.openings) -Expected $rawOpenings -Message 'Нормализованные проёмы разошлись с сырым входом.'
Assert-Equal -Actual @($request.source_geometry.lines).Count -Expected ([int] $assertions.integrity.source_line_count) -Message 'Не совпало количество стеновых линий.'
Assert-Equal -Actual @($request.openings).Count -Expected ([int] $assertions.integrity.opening_count) -Message 'Не совпало количество проёмов.'
Assert-Equal -Actual @($result.courses).Count -Expected ([int] $assertions.integrity.course_count) -Message 'Не совпало количество венцов.'
Assert-Equal -Actual ([int] $request.source_geometry.line_count) -Expected @($request.source_geometry.lines).Count -Message 'Объявленное количество стеновых линий неверно.'
Assert-Equal -Actual ([int] $request.opening_count) -Expected @($request.openings).Count -Message 'Объявленное количество проёмов неверно.'

$courseZ = @($result.courses | ForEach-Object { [int] $_.z_mm })
$courseCounts = @($result.courses | ForEach-Object { [int] $_.block_count })
Assert-JsonEqual -Actual $courseZ -Expected @($assertions.integrity.course_z_mm | ForEach-Object { [int] $_ }) -Message 'Не совпали отметки венцов.'
Assert-JsonEqual -Actual $courseCounts -Expected @($assertions.characterization.blocks_per_course | ForEach-Object { [int] $_ }) -Message 'Не совпали характеристические количества блоков.'

$groupedBlocks = @($result.courses | ForEach-Object { $_.blocks })
Assert-Equal -Actual $groupedBlocks.Count -Expected ([int] $result.total_block_count) -Message 'Сумма блоков по венцам не совпала с итогом.'
Assert-Equal -Actual $groupedBlocks.Count -Expected ([int] $assertions.characterization.total_block_count) -Message 'Результат разошёлся с характеристическим контрактом.'

$expectedBlockFields = @($manifest.test_contract.comparison.block_fields | Sort-Object)
$expectedCutFields = @($manifest.test_contract.comparison.cut_fields | Sort-Object)
foreach ($block in $groupedBlocks) {
    Assert-JsonEqual -Actual @($block.PSObject.Properties.Name | Sort-Object) -Expected $expectedBlockFields -Message 'Наблюдаемый блок не соответствует объявленной форме.'
    foreach ($cut in @($block.cuts)) {
        Assert-JsonEqual -Actual @($cut.PSObject.Properties.Name | Sort-Object) -Expected $expectedCutFields -Message 'Запил наблюдаемого блока не соответствует объявленной форме.'
    }
}

$duplicatePlacements = @($groupedBlocks | Group-Object {
    '{0}|{1}|{2}|{3}' -f $_.floor, $_.posX, $_.posY, $_.angleRad
} | Where-Object Count -gt 1)
Assert-Equal -Actual $duplicatePlacements.Count -Expected 0 -Message 'В наблюдаемом выходе найдены блоки с одинаковой позицией, направлением и венцом.'

foreach ($course in $result.courses) {
    Assert-Equal -Actual @($course.blocks).Count -Expected ([int] $course.block_count) -Message "Не совпало количество блоков венца $($course.course_index)."
    $unexpectedFloors = @($course.blocks | Where-Object { [int] $_.floor -ne [int] $course.legacy_floor })
    Assert-Equal -Actual $unexpectedFloors.Count -Expected 0 -Message "В венце $($course.course_index) есть блоки другого legacy floor."
}

Assert-Equal -Actual @($assertions.normative.legacy_block_output_assertions).Count -Expected 0 -Message 'Старый выход не должен становиться нормативным оракулом.'
$actualNonRegularLines = @()
$gridStepMm = [int] $request.constants.standard_grid_step_mm
for ($lineIndex = 0; $lineIndex -lt @($request.source_geometry.lines).Count; $lineIndex++) {
    $line = $request.source_geometry.lines[$lineIndex]
    $dxMm = ([double] $line.posBX - [double] $line.posAX) * 1000
    $dyMm = ([double] $line.posBY - [double] $line.posAY) * 1000
    $lengthMm = [int] [math]::Round([math]::Sqrt(($dxMm * $dxMm) + ($dyMm * $dyMm)))
    $remainderMm = $lengthMm % $gridStepMm
    if ($remainderMm -ne 0) {
        $actualNonRegularLines += [ordered]@{
            line_index = $lineIndex
            source_volume_guid = $line.sourceVolumeGuid
            length_mm = $lengthMm
            remainder_mm = $remainderMm
        }
    }
}
$declaredNonRegularLines = @($assertions.normative.regular_640_strategy.non_regular_source_lines)
if ($declaredNonRegularLines.Count -eq 0) {
    throw 'Смешанная фикстура должна содержать свидетельства неприменимости простой сетки 640 мм.'
}
Assert-JsonEqual -Actual $declaredNonRegularLines -Expected $actualNonRegularLines -Message 'Свидетельства неприменимости простой сетки разошлись с входной геометрией.'

[pscustomobject]@{
    Fixture = $manifest.fixture_id
    Checksums = $listedPaths.Count
    SourceLines = @($request.source_geometry.lines).Count
    Openings = @($request.openings).Count
    Courses = @($result.courses).Count
    ObservedBlocks = $groupedBlocks.Count
    NormativeLegacyBlockAssertions = @($assertions.normative.legacy_block_output_assertions).Count
    Status = 'PASS'
}
