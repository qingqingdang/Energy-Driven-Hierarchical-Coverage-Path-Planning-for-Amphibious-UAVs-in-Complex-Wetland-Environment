
# 🚀 项目快速上手指南 (Quick Start)

欢迎使用 **Energy-Driven-Hierarchical-Coverage-Path-Planning-for-Amphibious-UAVs-in-Complex-Wetland-Environment** 项目。请按照以下步骤配置你的运行环境。

### 1. 前置条件
确保你的电脑上已安装以下软件之一：
*   **Anaconda** 
*   **Git**

### 2. 获取代码
由于这是私有仓库，请先确保你已被添加为 Collaborator，然后克隆项目：
```bash
git clone https://github.com/Big-Black-Sheep/Energy-Aware-Cooperative-Coverage-Path-Planning-for-Amphibious-UAV-Swarms.git
cd Energy-Aware-Cooperative-Coverage-Path-Planning-for-Amphibious-UAV-Swarms
```

### 3. 一键配置环境
在项目根目录下（即包含 `environment.yml` 的地方），打开终端并运行：
```bash
conda env create -f environment.yml
```
> **注意**：这一步会自动读取 yml 文件中的依赖并创建一个新的虚拟环境。创建过程可能需要几分钟，请耐心等待。

### 4. 激活环境
环境安装完成后，激活它：
```bash
conda activate Global_Path_Planning
```
*(注意：如果 `environment.yml` 里的环境名称不同，请以 yml 第一行的 name 为准)*

### 5. 运行项目
现在你可以运行主程序了：
```bash
python main.py
```

---

### 💡 常见问题 (Tips for Collaborators)

*   **更新环境**：如果以后作者更新了 `environment.yml`，你只需要在激活环境后运行：
    `conda env update -f environment.yml --prune`
*   **缺少 LKH 求解器**：项目中包含 `data/LKH-3.exe`。如果是在 Linux/Mac 上运行，可能需要重新编译对应平台的 LKH 二进制文件并替换。
*   **路径问题**：项目内的路径建议使用相对路径，以确保代码在不同电脑上克隆后都能直接运行。

---


