rem SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
rem
rem SPDX-License-Identifier: MIT

@echo off

rem -------------------------------------------------------------------------------------------
rem Check if both kicad-cli.exe and tar.exe can be found and used
rem -------------------------------------------------------------------------------------------
where kicad-cli >nul 2>&1
if errorlevel 1 (
    echo Error: kicad-cli not found. Make sure it is installed and in PATH.
    exit /b 1
)

tar --version >nul 2>&1
if errorlevel 1 (
    echo Error: tar not found. It should be built-in with Windows.
    exit /b 1
)

rem -------------------------------------------------------------------------------------------
rem Declare script variables, delete and re-create the production files directory
rem -------------------------------------------------------------------------------------------
set "PROJECT_NAME=leany"
set "PROD_PATH=production_files"
set "kicadCPLfile=%PROD_PATH%\CPL-KICAD.csv"
set "jlcpcbCPLfile=%PROD_PATH%\CPL-JLCPCB.csv"
set "kicadBOMfile=%PROD_PATH%\BOM-KiCAD.csv"
set "jlcpcbBOMfile=%PROD_PATH%\BOM-JLCPCB.csv"
RMDIR /S /Q %PROD_PATH%
MKDIR %PROD_PATH%

rem -------------------------------------------------------------------------------------------
rem Create the Gerber zip and drill files
rem -------------------------------------------------------------------------------------------
echo Creating Gerber and drill files
kicad-cli pcb export gerbers --output %PROD_PATH% --board-plot-params %PROJECT_NAME%.kicad_pcb
if errorlevel 1 exit /b 1

kicad-cli pcb export drill --output %PROD_PATH% %PROJECT_NAME%.kicad_pcb
if errorlevel 1 exit /b 1

echo Zipping the gerber and drill files
tar -cvf JCLPCB_gerber.zip %PROD_PATH%
move JCLPCB_gerber.zip %PROD_PATH%
del %PROD_PATH%\*.g* %PROD_PATH%\*.drl

rem -------------------------------------------------------------------------------------------
rem Create the BOM file
rem -------------------------------------------------------------------------------------------
echo
echo Create BOM file and trim libraries names from footprints
kicad-cli sch export bom --output %kicadBOMfile% --preset JLCPCB --format-preset CSV %PROJECT_NAME%.kicad_sch
if errorlevel 1 exit /b 1

rem 1. Extract the KiCAD BOM file and its CSV header
rem 2. Trim the library names (including ':') in the third column
rem 3. Export the result to the JLCPCB BOM file
powershell -NoProfile -Command ^
    "$kicadBOMfile = '%kicadBOMfile%';" ^
    "$jlcpcbBOMfile = '%jlcpcbBOMfile%';" ^
    "$origHeader = Get-Content -Path $kicadBOMfile -Encoding UTF8 -TotalCount 1;" ^
    "$rows = Import-Csv -Path $kicadBOMfile -Delimiter ',' -Encoding UTF8;" ^
    "if ($rows.Count -ge 1) {" ^
    "  $footprintColumn = $rows[0].psobject.Properties.Name[2];" ^
    "  foreach ($row in $rows) {" ^
    "    $footprintName = [string]$row.$footprintColumn;" ^
    "    if ($footprintName -match '^[^:]*:(.*)$') {" ^
    "      $row.$footprintColumn = $Matches[1];" ^
    "    }" ^
    "  }" ^
    "}" ^
    "$rows | Export-Csv -Path $jlcpcbBOMfile -NoTypeInformation;"

del "%kicadBOMfile%"

rem -------------------------------------------------------------------------------------------
rem Create the POS file with the proper header
rem -------------------------------------------------------------------------------------------
echo
echo Creating POS file
kicad-cli pcb export pos --output %kicadCPLfile% --side front --format csv --units mm --use-drill-file-origin %PROJECT_NAME%.kicad_pcb
if errorlevel 1 exit /b 1

echo
echo Replace the KiCAD CPL header by the JLCPCB one
setlocal enabledelayedexpansion
set "search=Ref,Val,Package,PosX,PosY,Rot,Side"
set "replace=Designator,Val,Package,Mid X,Mid Y,Rotation,Layer"

(for /f "tokens=*" %%a in ('type "%kicadCPLfile%" ^| findstr /n "^"') do (
    set "line=%%a"
    set "line=!line:*:=!"

    if defined line (
        set "line=!line:%search%=%replace%!"
        echo(!line!
    ) else echo.
)) > "%jlcpcbCPLfile%"

del "%kicadCPLfile%"
echo %kicadCPLfile% replaced by %jlcpcbCPLfile%
endlocal

rem -------------------------------------------------------------------------------------------
rem Create the STL file
rem -------------------------------------------------------------------------------------------
echo Creating STL file
kicad-cli pcb export stl --output %PROD_PATH%\%PROJECT_NAME%.stl --force --subst-models --no-dnp --drill-origin %PROJECT_NAME%.kicad_pcb
if errorlevel 1 exit /b 1
