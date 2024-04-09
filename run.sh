./bin/main.exe $1 $2

# If $3 (speed) is provided, then record the result mp4
if [ $# -eq 3 ]
then
    python3 scripts/visualization.py --solver $1 --instance $2 --record --speed $3 
else
    python3 scripts/visualization.py --solver $1 --instance $2 --speed 1.0
fi
