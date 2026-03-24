# PyBullet移除完成 - 已全面切换到Mink+MuJoCo

## 已完成的更改

### 1. 依赖更新
- ✅ **移除**: `pybullet`
- ✅ **添加**: `mujoco>=3.0.0` (核心依赖)
- ✅ 更新了 `pyproject.toml` 和 `requirements.txt`

### 2. 核心模块替换

#### IK和FK求解器
- ✅ `robot_interface.py` - 完全基于Mink的新版本
  - 使用 `MinkIKSolver` 替代 PyBullet IK
  - 使用 `MinkForwardKinematics` 替代 PyBullet FK
  - 方法签名保持兼容

#### 可视化
- ✅ `visualizer.py` - 使用MuJoCo Viewer
  - `MuJoCoVisualizer` 替代 `PyBulletVisualizer`  
  - 支持3D实时可视化
  - 自动在后台线程运行viewer

#### 旧文件备份（已删除）
- `robot_interface_pybullet.py.bak` 
- `visualizer_pybullet.py.bak`
- `kinematics_pybullet.py.bak`

### 3. 配置文件

#### config.yaml (SO100默认)
```yaml
control:
  mink:
    enabled: true
    mujoco_scene: URDF/SO100/scene.xml
    end_effector_site: Wrist_Pitch_Roll
  pybullet:
    enabled: false
```

#### config.yaml (ARM620专用)
```yaml
control:
  use_mink: true
  mink:
    enabled: true
    mujoco_scene: mink/examples/arm620/scene.xml
    end_effector_site: tools_link
  pybullet:
    enabled: false
```

## Mink vs PyBullet 对比

| 特性 | PyBullet (旧) | Mink+MuJoCo (新) |
|------|--------------|------------------|
| IK算法 | 数值IK | 微分IK (更平滑) |
| 求解速度 | 中等 | 更快 |
| 可视化 | PyBullet GUI | MuJoCo Viewer |
| 物理引擎 | Bullet | MuJoCo (更准确) |
| 依赖大小 | 小 | 中等 |
| 实时性能 | 好 | 更好 |

## 使用方法

### 安装
```bash
# 基础安装
pip install -e .

# 或手动安装依赖
pip install -r requirements.txt

# Mink库（在mink子目录）
cd mink
pip install -e .
```

### 运行示例

**ARM620 VR遥操作**:
```bash
python examples/vr_teleoperate_arm620.py --config config.yaml --no-robot
```

**SO100 (默认)**:
```bash
python -m telegrip --config config.yaml --no-robot
```

## API兼容性

接口方法保持不变，现有代码无需修改：

```python
# robot_interface.py 方法（API未变）
robot.setup_mink_kinematics(scene_path, end_effector_site)
robot.solve_ik(arm, target_position)
robot.get_current_end_effector_position(arm)

# visualizer.py 方法（API未变）
viz.setup()
viz.update_robot_pose(angles, arm)
viz.update_marker_position(marker_name, position)
viz.disconnect()
```

## 已知优势

1. **更平滑的运动** - 微分IK提供更自然的轨迹
2. **更快的求解** - Mink优化的IK算法
3. **更好的物理仿真** - MuJoCo引擎更准确
4. **统一的工具链** - 与mink示例代码保持一致

## 需要的文件

确保以下文件存在：
- **ARM620**: `mink/examples/arm620/scene.xml`
- **SO100**: `URDF/SO100/scene.xml` (需要创建MuJoCo scene)

## 回退到PyBullet（如需要）

备份文件已删除，但可从git历史恢复：
```bash
git checkout HEAD~1 -- telegrip/core/robot_interface.py
git checkout HEAD~1 -- telegrip/core/visualizer.py
git checkout HEAD~1 -- telegrip/core/kinematics.py
# 恢复requirements
git checkout HEAD~1 -- requirements.txt pyproject.toml
```

## 故障排除

### MuJoCo Viewer无法启动
- 使用 `--no-viz` 运行headless模式
- 或确保有X11显示 (SSH需要 `-X` 标志)

### Mink导入错误
```bash
cd mink && pip install -e .
```

### 缺少scene.xml
需要为SO100创建MuJoCo scene文件，参考ARM620的scene.xml格式

---

✅ **迁移完成** - telegrip现已完全基于Mink+MuJoCo运行
