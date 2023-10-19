:: create the conda environment as a subdirectory
conda env create -p .conda/freecad -f conda/conda-env.yaml

:: add the environment subdirectory to the conda configuration
conda config --add envs_dirs $PWD/.conda
conda config --set env_prompt '({name})'

:: install the FreeCAD dependencies into the environment
conda run --live-stream -n freecad conda devenv -f conda/environment.devenv.yml
