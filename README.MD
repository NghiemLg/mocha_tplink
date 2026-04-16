# MOCHA Simulation Package

Simulation for 1 UGV (Jackal) + 1 UAV (Titan) on Single Laptop

## 📖 Documentation

**→ SEE README.md FOR COMPLETE SETUP GUIDE**

## 🚀 Quick Start

```bash
cd ~/swarm_ws && source devel/setup.bash
roslaunch mocha_tplink mocha_sim_single_all.launch
```

## 📁 Key Files

**Launch:**
- `launch/mocha_sim_single_all.launch` - Both robots (DỄ NHẤT)
- `launch/mocha_sim_single.launch` - One robot

**Config:**
- `config/robot_configs_sim.yaml` - Robot definitions
- `config/topic_configs_sim.yaml` - Topic sync
- `config/radio_configs.yaml` - ZMQ parameters

**Scripts:**
- `scripts/quick_setup_single.sh` - Auto setup

---

👉 **READ README.md FOR EVERYTHING**
