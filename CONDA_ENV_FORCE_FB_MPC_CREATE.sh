source ~/CONDA_INIT.sh
FORCE_FB_MPC_ENV=${1:-"./.conda_envs/force_fb_mpc"}
if [ ! -d "$FORCE_FB_MPC_ENV" ]
then
	conda env create -f ./conda/env.yaml --prefix $FORCE_FB_MPC_ENV
	conda config --set env_prompt '({name})'
	conda info --envs
    # conda init # -> This will add auto conda init script to ~/.bashrc
	conda activate $FORCE_FB_MPC_ENV
else
	echo "$FORCE_FB_MPC_ENV already exists"
fi
