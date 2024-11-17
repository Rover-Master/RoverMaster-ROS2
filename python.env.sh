source ".env/bin/activate"
PATHS=$(python3 -c "
import sysconfig;
paths = sysconfig.get_paths();
print(paths['data'], paths['purelib'], sep=':', end='')
")
export PYTHONPATH="$PATHS:$PYTHONPATH"
alias PYTHONPATH="echo $PYTHONPATH | tr ':' '\n'"
