#!/bin/sh

KICAD_HEADER='Ref,Val,Package,PosX,PosY,Rot,Side'
JLCPCB_HEADER='Designator,Val,Package,Mid X,Mid Y,Rotation,Layer'
PROD_PATH=production_files
GBR_PATH=gerber

echo " "
echo "Creating production files directory"
mkdir -pv $PROD_PATH
rm -rf $PROD_PATH/*
mkdir -pv $GBR_PATH
rm -rf $GBR_PATH/*

echo " "
echo "Creating Gerber and drill files"
kicad-cli pcb export gerbers --output $GBR_PATH/ --board-plot-params leany.kicad_pcb
kicad-cli pcb export drill --output $GBR_PATH/ leany.kicad_pcb
zip -j $PROD_PATH/$GBR_PATH-JLCPCB.zip $GBR_PATH/*

echo " "
echo "Create BOM file and trim libraries names from footprints"
kicad-cli sch export bom --output $PROD_PATH/BOM-JLCPCB.csv --preset JLCPCB --format-preset CSV leany.kicad_sch
awk -F',' 'BEGIN {OFS=","} NR==1 {print; next} {sub(/.*:/, "", $3); print}' $PROD_PATH/BOM-JLCPCB.csv > $PROD_PATH/cleaned_bom.csv
mv $PROD_PATH/cleaned_bom.csv $PROD_PATH/BOM-JLCPCB.csv

echo " "
echo "Creating POS file"
kicad-cli pcb export pos --output $PROD_PATH/CPL-JLCPCB.csv --side front --format csv --units mm --use-drill-file-origin leany.kicad_pcb
sed -i "s/$KICAD_HEADER/$JLCPCB_HEADER/g" $PROD_PATH/CPL-JLCPCB.csv
