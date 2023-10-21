:: create the conda environment as a subdirectory
mamba env create -p .conda/freecad -f conda/conda-env.yaml

:: add the environment subdirectory to the conda configuration
mamba config --add envs_dirs $PWD/.conda
mamba config --set env_prompt '({name})'

:: install the FreeCAD dependencies into the environment
mamba run --live-stream -n freecad mamba devenv -f conda/environment.devenv.yml
