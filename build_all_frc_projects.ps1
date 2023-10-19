$Depth = 1
$Levels = '/*' * $Depth

$ErrorActionPreference = "Stop"

Get-ChildItem -Directory "./$Levels" -Exclude ".github" |
    ForEach-Object {
        Push-Location $_.FullName
        echo "Building example $_"
        ./gradlew build
        if (-not $?) {
            throw "Example $_ failed to build"
        }
        Pop-Location
    }