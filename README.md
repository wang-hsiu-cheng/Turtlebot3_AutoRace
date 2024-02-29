# TB3

## How to use

### install pkg
 - `sudo apt install ros-noetic-gmapping`
 - `sudo apt install ros-noetic-map-server`
 - `sudo apt install ros-noetic-navigation`

### environment
This repo is recommended pushing From WSL Ubuntu20.04.  
And Rasbpi 4B pulls this repo.

## About Testing

### nevigation stack testing
- `export TURTLEBOT3_MODEL=burger`
- `roslaunch turtlebot3_gazebo turtlebot3_world.launch`
- open new terminal
- `source devel/setup.bash`
- `roslaunch tb3_navigation move_base.launch`

### main program missions flow testing
- run testing node(not ready yet)

### camera testing
- [check out this repo.](https://github.com/wang-hsiu-cheng/openCV)
- The original version of the openCV programs used in this project are there.

## Development rules and Matters needing attention

### Git
 - 進行更動前請先 `git pull` 確認目前位於最新的主分支上，若已經改動程式碼才發現忘記開新分支，可以善用 `git stash`
 - 請確認修改完的程式要可以正確執行 `catkin_make` 編譯完成沒有錯誤或警告後再 Commit
 - 請勿將程式碼直接推到主分支 `master`，任何更改請一律開新分支再發送 PR
 - 分支名稱可以按照功能以及內容命名，例如 `feature/background_music` 、 `fix/change_background_music`
   - `feature` : 新增新的內容
   - `fix` : 修改既有的內容
 - 版本落後主分支太多的分支請不要繼續使用，請以最新版的主分支為基準再開一個新分支
 - Commit Message 編寫方式以及 branch 命名方式可以參考[這個連結](https://hackmd.io/@immortalmice/S1FafvL6U)。不強制要求，但是寫清楚可以方便其他人了解更改內容與 debug

### Coding Style
不強制，但提供一些規範給大家參考

#### Typesetting
  - 建議開啟 VScode 自動排版功能
    1. 進入 Setting
    2. 搜尋 Format
    3. 將 Format On Save 選項打勾

#### Files and Params' Naming Rules
 - fileName 與 class 使用 Pascal Case
   - `Wheel.cpp`
   - `class Cameras{}`
 - public variable 與 function name 使用 Pascal Case
   - `public int MaxHealthPoint;`
   - `void ResetHealthPoint(){}`
 - variable 與 function parameters 使用 Camel Case
   - `void SetHealthPoint(int healthPoint){}`
   - `private int currentHealthPoint;`
 - 避免使用簡稱
   - `int spd` => `int speed`
   - `GameObject tgt` => `GameObject target`
 - 布林變數或回傳 bool 的 function 以 `is` 或 `has` 作為開頭
   - `private bool _isDied;`
   - `private bool HasBuff(){}`

