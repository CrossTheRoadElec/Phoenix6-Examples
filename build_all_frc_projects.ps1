$Depth = 1
$Levels = '/*' * $Depth

$ErrorActionPreference = "Stop"

$exclude_list = @(".github", "automation")
$starting_location = Get-Location

Get-ChildItem -Directory "./$Levels" -Exclude $exclude_list |
    ForEach-Object {
        Push-Location $_.FullName
        echo "Building example $_"
        ./gradlew build
        if (-not $?) {
            throw "Example $_ failed to build"
        }
        Pop-Location
    }
