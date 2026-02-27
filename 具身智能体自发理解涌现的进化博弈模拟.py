"""
具身智能体自发理解涌现的进化博弈模拟
核心机制：智能体在能量驱动的环境中，通过策略变异、自然选择和多代遗传，进化出趋利避害的适应性行为。
这种行为策略的优化与固化，可类比为一种‘自发理解’的雏形。
"""
import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Environment:
    """模拟一个简单的二维网格世界，包含能量源和危险区。"""
    def __init__(self, size=20, num_energy_sources=5, num_dangers=3):
        self.size = size
        # 0: 空地, 1: 能量源, -1: 危险区
        self.grid = np.zeros((size, size), dtype=int)
        # 随机放置能量源（正收益）
        for _ in range(num_energy_sources):
            x, y = random.randint(0, size-1), random.randint(0, size-1)
            self.grid[x, y] = 1
        # 随机放置危险区（负收益）
        for _ in range(num_dangers):
            x, y = random.randint(0, size-1), random.randint(0, size-1)
            # 确保不覆盖能量源
            while self.grid[x, y] != 0:
                x, y = random.randint(0, size-1), random.randint(0, size-1)
            self.grid[x, y] = -1

    def get_reward(self, x, y):
        """在位置(x, y)上获得收益。"""
        if 0 <= x < self.size and 0 <= y < self.size:
            return self.grid[x, y]
        return 0  # 走出边界无收益

class EmbodiedAgent:
    """具身智能体，拥有一个简单的内部策略（感知-动作映射）。"""
    def __init__(self, x, y, strategy=None, max_energy=50):
        self.x = x
        self.y = y
        self.energy = max_energy
        self.max_energy = max_energy
        self.alive = True
        self.offspring_count = 0
        # 策略：一个长度为4的数组，对应[上, 下, 左, 右]四个方向的“偏好值”
        # 智能体根据偏好值选择移动方向（softmax决策）
        if strategy is None:
            self.strategy = np.random.randn(4)  # 初始随机策略
        else:
            self.strategy = strategy.copy()

    def perceive_and_act(self, local_view):
        """
        根据局部感知（目前仅用自身坐标）和内部策略决定动作。
        动作：0上, 1下, 2左, 3右。
        """
        # 简单策略：直接根据内部策略的偏好值做softmax决策
        preferences = self.strategy
        exp_prefs = np.exp(preferences - np.max(preferences))  # 数值稳定性
        prob_dist = exp_prefs / exp_prefs.sum()
        action = np.random.choice(4, p=prob_dist)
        return action

    def move(self, action, env_size):
        """执行移动，并处理边界。"""
        dx = [0, 0, -1, 1]
        dy = [-1, 1, 0, 0]
        new_x = self.x + dx[action]
        new_y = self.y + dy[action]
        # 简单边界处理：撞墙则不动
        if 0 <= new_x < env_size and 0 <= new_y < env_size:
            self.x, self.y = new_x, new_y

    def update_energy(self, reward, movement_cost=1):
        """更新能量，并检查生存状态。"""
        self.energy += reward - movement_cost
        if self.energy <= 0:
            self.alive = False
        elif self.energy > self.max_energy:
            self.energy = self.max_energy

    def can_reproduce(self, reproduction_threshold):
        """检查是否满足繁殖条件。"""
        return self.energy >= reproduction_threshold and self.alive

    def reproduce(self, mutation_rate=0.1):
        """繁殖一个后代，策略遗传并可能发生变异。"""
        self.energy -= self.max_energy * 0.6  # 繁殖消耗能量
        self.offspring_count += 1
        child_strategy = self.strategy + np.random.randn(4) * mutation_rate
        return EmbodiedAgent(self.x, self.y, child_strategy, self.max_energy)

class EvolutionaryGame:
    """管理整个进化博弈模拟。"""
    def __init__(self, env_size=20, num_agents=20, max_generations=100):
        self.env = Environment(size=env_size)
        self.env_size = env_size
        self.agents = [EmbodiedAgent(random.randint(0, env_size-1),
                                     random.randint(0, env_size-1),
                                     max_energy=50)
                       for _ in range(num_agents)]
        self.max_generations = max_generations
        self.generation = 0
        self.history = {'avg_energy': [], 'population': [], 'avg_strategy_norm': []}

    def run_generation(self):
        """运行一个世代。"""
        # 每个智能体进行若干步的感知-行动循环
        steps_per_gen = 10
        movement_cost = 1
        reproduction_threshold = 40

        for _ in range(steps_per_gen):
            random.shuffle(self.agents)  # 随机顺序更新
            for agent in self.agents[:]:  # 遍历列表副本
                if not agent.alive:
                    continue
                # 感知（这里简化，只使用自身位置，更复杂的模型可以加入局部视觉）
                action = agent.perceive_and_act(None)
                agent.move(action, self.env_size)
                reward = self.env.get_reward(agent.x, agent.y)
                agent.update_energy(reward, movement_cost)

        # 繁殖阶段
        new_agents = []
        for agent in self.agents:
            if agent.alive:
                new_agents.append(agent)
                if agent.can_reproduce(reproduction_threshold):
                    child = agent.reproduce(mutation_rate=0.15)
                    new_agents.append(child)
        self.agents = new_agents

        # 记录数据
        alive_agents = [a for a in self.agents if a.alive]
        if alive_agents:
            self.history['avg_energy'].append(np.mean([a.energy for a in alive_agents]))
            self.history['population'].append(len(alive_agents))
            self.history['avg_strategy_norm'].append(np.mean([np.linalg.norm(a.strategy) for a in alive_agents]))
        else:
            self.history['avg_energy'].append(0)
            self.history['population'].append(0)
            self.history['avg_strategy_norm'].append(0)

        self.generation += 1
        return len(alive_agents) > 0

    def run_simulation(self):
        """运行整个模拟直到最大世代数或种群灭绝。"""
        for gen in range(self.max_generations):
            if not self.run_generation():
                print(f"种群在第 {gen+1} 代灭绝。")
                break
        else:
            print(f"完成 {self.max_generations} 代模拟。")
        return self.history

    def visualize(self, history):
        """可视化进化过程。"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        gens = range(self.generation)

        # 1. 平均能量变化
        axes[0, 0].plot(gens, history['avg_energy'], 'b-', linewidth=2)
        axes[0, 0].set_xlabel('世代')
        axes[0, 0].set_ylabel('平均能量')
        axes[0, 0].set_title('种群平均能量随世代变化')
        axes[0, 0].grid(True, alpha=0.3)

        # 2. 种群数量变化
        axes[0, 1].plot(gens, history['population'], 'g-', linewidth=2)
        axes[0, 1].set_xlabel('世代')
        axes[0, 1].set_ylabel('种群数量')
        axes[0, 1].set_title('种群数量随世代变化')
        axes[0, 1].grid(True, alpha=0.3)

        # 3. 平均策略范数（策略复杂性/特异性指标）
        axes[1, 0].plot(gens, history['avg_strategy_norm'], 'r-', linewidth=2)
        axes[1, 0].set_xlabel('世代')
        axes[1, 0].set_ylabel('平均策略范数')
        axes[1, 0].set_title('策略特异性随世代变化')
        axes[1, 0].grid(True, alpha=0.3)

        # 4. 最终世代智能体与环境的散点图
        axes[1, 1].clear()
        # 绘制环境
        for x in range(self.env_size):
            for y in range(self.env_size):
                val = self.env.grid[x, y]
                color = 'white'
                if val == 1:
                    color = 'gold'
                    axes[1, 1].scatter(x, y, c=color, s=100, marker='*', edgecolors='orange', linewidths=1, label='能量源' if x==0 and y==0 else "")
                elif val == -1:
                    color = 'darkred'
                    axes[1, 1].scatter(x, y, c=color, s=150, marker='X', label='危险区' if x==0 and y==0 else "")
        # 绘制智能体
        alive_agents = [a for a in self.agents if a.alive]
        if alive_agents:
            xs = [a.x for a in alive_agents]
            ys = [a.y for a in alive_agents]
            energies = [a.energy for a in alive_agents]
            sc = axes[1, 1].scatter(xs, ys, c=energies, cmap='viridis', s=50, alpha=0.8, label='智能体', edgecolors='black')
            plt.colorbar(sc, ax=axes[1, 1], label='个体能量')
        axes[1, 1].set_xlim(-1, self.env_size)
        axes[1, 1].set_ylim(-1, self.env_size)
        axes[1, 1].set_xlabel('X 坐标')
        axes[1, 1].set_ylabel('Y 坐标')
        axes[1, 1].set_title(f'最终世代 (Gen {self.generation}) 环境与智能体分布')
        axes[1, 1].grid(True, alpha=0.3)
        # 合并图例
        handles, labels = axes[1, 1].get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        axes[1, 1].legend(by_label.values(), by_label.keys())

        plt.suptitle('具身智能体自发理解的进化博弈模拟', fontsize=16)
        plt.tight_layout()
        plt.show()

# ========== 模拟执行与结果展示 ==========
if __name__ == "__main__":
    # 初始化并运行模拟
    game = EvolutionaryGame(env_size=15, num_agents=30, max_generations=80)
    history = game.run_simulation()

    # 打印最终世代的统计信息
    alive_agents = [a for a in game.agents if a.alive]
    if alive_agents:
        print(f"\n模拟结束于第 {game.generation} 代。")
        print(f"存活个体数: {len(alive_agents)}")
        print(f"平均能量: {np.mean([a.energy for a in alive_agents]):.2f}")
        print(f"平均策略向量范数: {np.mean([np.linalg.norm(a.strategy) for a in alive_agents]):.3f}")
        # 示例查看一个存活个体的策略
        sample_agent = alive_agents[0]
        print(f"示例个体策略向量 [上,下,左,右]: {sample_agent.strategy}")
        # 计算其策略的明确性（softmax概率的熵）
        prob = np.exp(sample_agent.strategy)
        prob = prob / prob.sum()
        entropy = -np.sum(prob * np.log(prob + 1e-10))
        print(f"示例个体策略熵 (越低越明确): {entropy:.3f}")
    else:
        print("所有个体均已死亡。")

    # 可视化结果
    game.visualize(history)
