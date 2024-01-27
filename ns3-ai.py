from ctypes import *
from py_interface import *
from model import ReinforceAgent
import numpy as np
import time
import os
from tqdm import tqdm
import torch
import torch.nn.functional as F
from torch.optim import Adam

class AdversarialAttackerEnv(Structure):
    _pack_ = 1
    _fields_ = [
        ('mcs', c_ubyte),
        ('max_mcs', c_ubyte),
        ('time', c_double),
        ('throughput', c_double),
        ('snr', c_double)
    ]

class AdversarialAttackerAct(Structure):
    _pack_ = 1
    _fields_ = [
        ('shouldJam', c_bool),
    ]

class AdversarialAttackerContainer:
    use_ns3ai = True

    def __init__(self, uid: int = 2337, model_name='',
                 retrain=False,
                 jam_packets=500,
                 tpt_threshold=30,
                 scene=1,
                 no_jam=False,
                 heur=False) -> None:
        self.rl = Ns3AIRL(uid, AdversarialAttackerEnv, AdversarialAttackerAct)
        self.agent = ReinforceAgent(12, 2)  # Input dimension and output dimension
        self.optimizer = Adam(self.agent.parameters(), lr=1e-4)
        self.isFinished = False
        self.startTime = time.time()
        self.last_state = {'throughput': 0, 'mcs': 0, 'time': 0, 'snr': 0}
        self.last_action = {'jam': False}
        self.isInference = not retrain
        self.isFirstEpisode = True
        self.target_throughput = tpt_threshold
        self.attack_packet_count = 0
        self.max_attack_packet_per_sec = jam_packets
        self.isAiOff = no_jam
        self.scene = scene
        self.heur = heur

        if os.path.exists(model_name):
            self.load_model(model_name, retrain)

    def train_ai_jamming(self, env: AdversarialAttackerEnv, act: AdversarialAttackerAct) -> AdversarialAttackerAct:
        if self.isInference:
            return self.inference_ai_jamming(env, act)

        if self.isFirstEpisode:
            self.isFirstEpisode = False
            self.last_state = {'throughput': env.throughput, 'mcs': env.mcs, 'time': env.time, 'snr': env.snr}
            act.shouldJam = False
            return act

        current_state = [env.throughput, self.target_throughput, env.mcs, self.attack_packet_count, env.snr]
        action = self.agent.choose_action(current_state)  
        act.shouldJam = action == 1

        throughput_deviation = -0.5 * abs(env.throughput - self.target_throughput)
        jamming_efficiency = -0.5 * max(0, self.attack_packet_count - self.max_attack_packet_per_sec)
        reward = throughput_deviation + jamming_efficiency
        self.agent.rewards.append(reward)  

        self.last_state = {'throughput': env.throughput, 'mcs': env.mcs, 'time': env.time, 'snr': env.snr}
        self.last_action = {'jam': act.shouldJam}
        self.attack_packet_count += 1 if act.shouldJam else 0

        self.agent.update()  

        return act

    def inference_ai_jamming(self, env: AdversarialAttackerEnv, act: AdversarialAttackerAct) -> AdversarialAttackerAct:
        current_state = [env.throughput, self.target_throughput, env.mcs, self.attack_packet_count, env.snr]
        action = self.agent.choose_action(current_state, epsilon=0.0)  
        act.shouldJam = action == 1
        return act

    def save_model(self, filename):
        self.agent.save_model(filename)

    def load_model(self, filename, retrain=False):
        self.agent.load_model(filename)
        if retrain:
            self.agent.train()
        else:
            self.agent.eval() 


if __name__ == '__main__':
    mempool_key, mem_size, memblock_key, iters_count, sim_runs = 1347, 4096, 2337, 0, 20

    for _ in range(sim_runs):
        rand_seed = np.random.randint(0, 100000)
        ns3Settings = {"apManager": "ns3::MinstrelHtWifiManager"}
        exp = Experiment(mempool_key, mem_size, 'ad', '../../', using_waf=False)
        exp.reset()
        c = AdversarialAttackerContainer(memblock_key, model_name='ai_jamming.pt', retrain=True)
        pro = exp.run(setting=ns3Settings, show_output=True)
        print("Running rate-control with settings:", ns3Settings)

        epochs, isTqdm = 3000, False
        progress = tqdm(total=epochs) if isTqdm else None

        while not c.rl.isFinish():
            with c.rl as data:
                if data is None:
                    break
                data.act = c.train_ai_jamming(data.env, data.act)
                if isTqdm:
                    progress.update(1)
                if iters_count > epochs:
                    c.save_model('best.ckpt')
                    print('Model saved')
                    break
                iters_count += 1

        if isTqdm:
            progress.close()
        pro.wait()
    del exp

