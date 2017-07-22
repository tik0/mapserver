#!/bin/bash
FILE="lem_oct_2016.txt.yml"
N_TH=${1}
OUT="lem_oct_2016_${N_TH}th.txt.yml"

# Process timestamp
echo -n "timestamp: [" > ${OUT}
cat ${FILE} | head -n1 | tr "," "\n" | sed "s/\ //g" | sed "s/\]//g" | sed "s/timestamp\:\[//g" | sed -n "1~${N_TH}p" | tr "\n" "," | sed "s/\,/\, /g" >> ${OUT}
echo "]" >> ${OUT}

# Process sec
echo -n "sec: [" >> ${OUT}
cat ${FILE} | head -n2 | tail -n1 | tr "," "\n" | sed "s/\ //g" | sed "s/\]//g" | sed "s/sec\:\[//g" | sed -n "1~${N_TH}p" | tr "\n" "," | sed "s/\,/\, /g" >> ${OUT}
echo "]" >> ${OUT}

# Process nsec
echo -n "nsec: [" >> ${OUT}
cat ${FILE} | tail -n1 | tr "," "\n" | sed "s/\ //g" | sed "s/\]//g" | sed "s/nsec\:\[//g" | sed -n "1~${N_TH}p" | tr "\n" "," | sed "s/\,/\, /g" >> ${OUT}
echo "]" >> ${OUT}

# Cleanup
sed -i "s/\,\ \]/\ \]/g" ${OUT}
 
