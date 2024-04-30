if ./bin/main.exe $1 $2
then
    python3 scripts/visualization_python3.8.py --solver $1 --instance $2 --speed 1.0
else
    echo "Failed!"
fi