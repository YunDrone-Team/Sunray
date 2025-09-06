tmux kill-session -t sunray_tmux
rostopic pub -1 /baton/stereo3_ctrl system_ctrl/algo_ctrl "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
algo_enable: false
algo_reboot: false
algo_reset: false"

