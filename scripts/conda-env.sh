#!/bin/bash
# ============================================================
# Auto detection and activation of conda environment
# ============================================================
# Author: Yuxuan Zhang
# Email : robotics@z-yx.cc
# License: MIT
# ============================================================
# Check for environment.(yml|yaml) file
if [ -f "environment.yml" ]; then
    CONDA_ENV_FILE="environment.yml"
elif [ -f "environment.yaml" ]; then
    CONDA_ENV_FILE="environment.yaml"
fi
# Try to activate conda environment for the environment file
if [ ! -z "${CONDA_ENV_FILE}" ]; then
    # Check if "conda" command is available
    if ! command -v conda &> /dev/null; then
        echo -e "\e[33m[WARN] \e[0;90m Conda not found on this system, but environment file exists."
    else
        CONDA_ENV=$(scripts/conda_env.py < ${CONDA_ENV_FILE})
        # Check for exit code
        if [ $? -eq 0 ]; then
            conda activate "${CONDA_ENV}" \
            && echo -e "\e[32m[INFO] \e[0;90m Activating Conda Env: ${CONDA_ENV}" \
            || echo -e "\e[31m[ERROR]\e[0;90m Error activating ${CONDA_ENV}"
        else
            echo -e "\e[31m[ERROR]\e[0;90m Error parsing ${CONDA_ENV_FILE}: ${CONDA_ENV}"
        fi
    fi
fi
# Append conda python paths
PYTHONPATH="${PYTHONPATH}:$(python -c "import sysconfig; print(sysconfig.get_paths()['purelib'])")"
LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:$(python -c "import sysconfig, os; print(os.path.dirname(sysconfig.get_paths()['stdlib']))")"
