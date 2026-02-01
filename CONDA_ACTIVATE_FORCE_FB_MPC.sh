#conda init -> This will add init script to ~/.bashrc
source ~/CONDA_INIT.sh
export PYTHONNOUSERSITE=True    # prevent using packages from base
conda activate ./.conda_envs/force_fb_mpc
