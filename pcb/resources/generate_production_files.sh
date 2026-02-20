# SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
#
# SPDX-License-Identifier: MIT

#!/bin/sh

set -e

PROJECT_NAME=leany
KICAD_HEADER='Ref,Val,Package,PosX,PosY,Rot,Side'
JLCPCB_HEADER='Designator,Val,Package,Mid X,Mid Y,Rotation,Layer'
PROD_PATH=production_files

echo " "
echo "Creating production files directory"
mkdir -pv $PROD_PATH
rm -rf $PROD_PATH/*

echo " "
echo "Creating Gerber and drill files"
kicad-cli pcb export gerbers --output $PROD_PATH/ --board-plot-params $PROJECT_NAME.kicad_pcb
kicad-cli pcb export drill --output $PROD_PATH/ $PROJECT_NAME.kicad_pcb
echo " "
echo Zipping the gerber and drill files
zip -jmv $PROD_PATH/JCLPCB_gerber.zip $PROD_PATH/*

echo " "
echo "Create BOM file and trim libraries names from footprints"
kicad-cli sch export bom --output $PROD_PATH/BOM-JLCPCB.csv --preset JLCPCB --format-preset CSV $PROJECT_NAME.kicad_sch
awk -F',' 'BEGIN {OFS=","} NR==1 {print; next} {sub(/.*:/, "", $3); print}' $PROD_PATH/BOM-JLCPCB.csv > $PROD_PATH/cleaned_bom.csv
mv $PROD_PATH/cleaned_bom.csv $PROD_PATH/BOM-JLCPCB.csv

echo " "
echo "Creating POS file"
kicad-cli pcb export pos --output $PROD_PATH/CPL-JLCPCB.csv --side front --format csv --units mm --use-drill-file-origin $PROJECT_NAME.kicad_pcb
sed -i "s/$KICAD_HEADER/$JLCPCB_HEADER/g" $PROD_PATH/CPL-JLCPCB.csv

echo " "
echo "Creating STL file"
export KICAD9_3DMODEL_DIR="/usr/share/kicad/3dmodels"
kicad-cli pcb export stl --output $PROD_PATH/$PROJECT_NAME.stl --force --subst-models --no-dnp --drill-origin $PROJECT_NAME.kicad_pcb
