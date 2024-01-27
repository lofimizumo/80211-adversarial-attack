# 802.11 Wifi Adversarial Attacking Algorithm

This project involves the implementation of an adversarial attacker in a network environment, utilizing the Reinforcement Learning (RL) approach with a specific focus on the REINFORCE algorithm. The goal is to learn and execute jamming strategies effectively.

## Description

The project integrates a Reinforcement Learning agent, specifically employing the REINFORCE algorithm, to interact with a network environment. The RL agent learns to decide whether to jam the network based on the current network state, aiming to disrupt communication strategically. 

## Features

- **RL Agent Implementation**: Utilizes the REINFORCE algorithm for policy gradient-based learning.
- **Network Environment Interaction**: Interfaces with an NS-3 simulated network environment.
- **Adaptive Learning**: Capable of adjusting the jamming strategy based on observed network states and rewards.

## Installation

To set up the project environment:

**Clone the repository:**
```bash
git clone https://github.com/lofimizumo/80211-adversarial-attack.git 
cd your-repo-name'''
```

## Usage

**Prerequisites**

    NS3-AI:
        Make sure you have NS3-AI installed.
        For installation guidelines, refer to NS3-AI Installation Guide.

    NS3:
        The code has been tested on NS3 version 3.36.


**Configuration**

You can adjust the simulation parameters in the config.py file:

    mempool_key: Unique key for memory pool.
    mem_size: Size of memory pool in bytes.
    memblock_key: Unique key for memory block within the memory pool.
    sim_runs: Number of simulation runs.

