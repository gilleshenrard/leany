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
zip -jmv $PROD_PATH/JLCPCB_gerber.zip $PROD_PATH/*

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
echo Create the PDF files
rm -f "$PROD_PATH/${PROJECT_NAME}_schematic.pdf" "$PROD_PATH/${PROJECT_NAME}_pcb_front.pdf" "$PROD_PATH/${PROJECT_NAME}_pcb_back.pdf"
kicad-cli sch export pdf --output "$PROD_PATH/${PROJECT_NAME}_schematic.pdf" ${PROJECT_NAME}.kicad_sch
kicad-cli pcb export pdf --mode-single --layers "F.Cu,F.Paste,F.Silkscreen,F.Mask,Edge.Cuts" --output "$PROD_PATH/${PROJECT_NAME}_pcb_front.pdf" ${PROJECT_NAME}.kicad_pcb
kicad-cli pcb export pdf --mirror --mode-single --layers "B.Cu,B.Paste,B.Silkscreen,B.Mask,Edge.Cuts" --output "$PROD_PATH/${PROJECT_NAME}_pcb_back.pdf" ${PROJECT_NAME}.kicad_pcb
pdfunite "$PROD_PATH/${PROJECT_NAME}_schematic.pdf" "$PROD_PATH/${PROJECT_NAME}_pcb_front.pdf" "$PROD_PATH/${PROJECT_NAME}_pcb_back.pdf" "$PROD_PATH/${PROJECT_NAME}.pdf"

echo " "
echo "Creating STEP file"
export KICAD9_3DMODEL_DIR="/usr/share/kicad/3dmodels"
kicad-cli pcb export step --output $PROD_PATH/$PROJECT_NAME.step --force --subst-models --drill-origin $PROJECT_NAME.kicad_pcb
