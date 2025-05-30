# 强化学习



## 定义
强化学习（Reinforcement Learning，RL），是指一类从（与环境）交互中不断学习的问题以及解决这类问题的方法. 强化学习问题可以描述为一个智能体从与环境的交互中不断学习以完成特定目标（比如取得最大奖励值）




## 基本元素
1. 状态s(state) 
2. 动作(action)
3. 策略 $\pi(a|s) = P(A_t=a | S_t=s)$
4. 在策略π和状态s时，采取行动后的价值（value），用vπ(s)表示
5. 奖励(reward) t时刻个体在状态St,采取的动作At,对应的奖励Rt+1会在t+1时刻得到
6. 环境的状态转化模型，在状态s下采取动作a,转到下一个状态s′的概率，$P_{ss'}^a$


在某个state下，agent依据policy，采取action，与environment交互，agent获得反馈reward。agent获得的reward会指导policy改进，在state下选择action。循环往复，policy不断被优化


## 马尔可夫性质
未来状态的条件概率分布仅依赖于当前状态


## MDP(Markov Decision Process)
![图片](https://pica.zhimg.com/v2-275e9fda3b9d1d8495a137fda77b550a_r.jpg)


状态的价值 $v_{\pi}(s) = \mathbb{E}_{\pi}(R_{t+1} + \gamma R_{t+2} + \gamma^2R_{t+3}+...|S_t=s) $
动作 $q_{\pi}(s,a) = \mathbb{E}_{\pi}(G_t|S_t=s, A_t=a) = \mathbb{E}_{\pi}(R_{t+1} + \gamma R_{t+2} + \gamma^2R_{t+3}+...|S_t=s,A_t=a)$
贝尔曼方程。这个式子告诉我们，一个状态的价值由该状态的奖励以及后续状态价值按一定的衰减比例联合组成
动作价值函数qπ(s,a)和状态价值函数vπ(s)的定义，我们很容易得到他们之间的转化关系 $v_{\pi}(s) = \sum\limits_{a \in A} \pi(a|s)q_{\pi}(s,a)$
状态价值函数是所有动作价值函数基于策略π的期望。通俗说就是某状态下所有状态动作价值乘以该动作出现的概率，最后求和，就得到了对应的状态价值
最优状态价值函数 $v_{*}(s) = \max_{a}q_{*}(s,a)$
最优动作价值函数
---

## DP
动态规划
求解马尔科夫决策过程的核心问题：预测和控制
对环境已知(model based)
预测：给定强化学习的6个要素：状态集S, 动作集A, 模型状态转化概率矩阵P, 即时奖励R，衰减因子γ,  给定策略π， 求解该策略的状态价值函数v(π)



### policy evaluation
基于随机策略的状态价值
### policy iteration
贪婪策略：个体在某个状态下选择的行为是其能够到达后续所有可能的状态中状态价值最大的那个状态

给定一个初始策略 π ，可以得到基于该策略的价值函数 vπ，而基于该价值函数又可以得到一个贪婪策略 π′ = greedy(vπ)。然后依据新的策略 π′ 又可以得到一个新的价值函数，并由此产生新的贪婪策略，如此反复进行，价值函数和策略均得到迭代更新，并最终收敛得到最优价值函数 v∗ 和最优策略 π∗

![1](https://images2018.cnblogs.com/blog/1042406/201808/1042406-20180812184148124-1485684702.jpg)

![1](https://i-blog.csdnimg.cn/blog_migrate/d1987e94eb950ea4567f7bb5f35b2035.png#pic_center)

左图是依据均一随机策略的第二次迭代后各状态价值函数，右图是根据左图各状态的价值绘制的贪婪策略方案。我们可以看到，相比于之前的均一随机策略，新的策略可以让 agent 在靠近终止状态的几个状态中有明确的行为，而不是之前的随机行为，这说明了新策略要比原策略优秀。而这个从均一随机策略下的价值函数中产生新的更优秀的贪婪策略的过程，就是一个策略改善的过程。



### value iteration
没有等到状态价值收敛才调整策略，而是随着状态价值的迭代及时调整策略, 这样可以大大减少迭代次数
每次更新倾向于贪婪法选择的最优策略对应的后续状态价值，这样收敛更快
不停地去迭代 Bellman Optimality Equation，到了最后，它能逐渐趋向于最佳的策略




## 蒙特卡洛法
不基于模型
通过采样若干经历完整的状态序列(episode)来估计状态的真实价值。序列必须是达到终点的。

要求某一个状态的状态价值，只需要求出所有的完整序列中该状态出现时候的收获再取平均值即可近似求解
预测：$G_t =R_{t+1} + \gamma R_{t+2} + \gamma^2 R_{t+3}+...  \gamma^{T-t-1}R_{T}$          average(G_t), s.t. St=S

控制：ϵ−贪婪法 使用1−ϵ的概率贪婪地选择目前认为是最大行为价值的行为，而用ϵ的概率随机的从所有可选行为中选择行为




## 时间差分（TD）方法




> (https://github.com/nndl/nndl.github.io)
> (https://www.cnblogs.com/pinard/p/9385570.html)
> (https://blog.csdn.net/qq_45832958/article/details/123188899)
> (https://www.youtube.com/watch?v=2pWv7GOvuf0&list=PLqYmG7hTraZDM-OYHWgPebj2MfCFzFObQ)