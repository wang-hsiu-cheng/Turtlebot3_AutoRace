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

### main program test
- if want test level 1 ~ level 5: `git chechout feature/wheel-and-camera-integrate`
- if only want test level 6(tunnel navigation), `git chechout Fix/navigation-and-main-program-connection`
1. use terminal to open gazebo simulation environment
   1. `export TURTLEBOT3_MODEL=burger`
   2. `roslaunch turtlebot3_gazebo turtlebot3_world.launch`
2. modify begin_state & reset_state in `auto.launch`
   - if you want to test level 2(turn sign), set `begin_state = 2` & `reset_state = 2`
   - if you want to test level 2 to level 4, set `begin_state = 2` & `reset_state = 4`
   - etc.
3. open new terminal and launch navigation programs
   1. `source devel/setup.bash`
   2. `roslaunch race auto.launch`
- NOTE:
because I'm testing in virtual machine that can't open camera of computer
So, all program about openCV are changed to photo reading mode
If you don't set the source photo path correct. openCV won't process any photo, and the program won't stop!!!
```cpp
// can only set direct path like below, relative path is NOT allowed! (IDK why, maybe a bug)
#define SOURCE_TURN_LEFT "/home/twang/pictureSource/turntest.jpg"
// and function will read the photo directly
img = imread(SOURCE_TURN_LEFT);
```
you can add an integer variable to limit the running times of while loops to prevent infinite loop like below
```cpp
int testCounter = 0;
do {
   testCounter ++;
   WHEEL::move_front(3, 0);
   VISION::takingPhoto(3);
} while (!VISION::isDetected && testCounter < 20);
```

### nevigation stack testing
1. use terminal to open gazebo simulation environment
   1. `export TURTLEBOT3_MODEL=burger`
   2. `roslaunch turtlebot3_gazebo turtlebot3_world.launch`
2. set the navigation goal in `move_base.launch`
   1. `<param name="/goal_x" type="double" value="0.0" />`
   2. `<param name="/goal_y" type="double" value="0.5" />`
   - note that thay're global position in gazebo world, not local position!!!
3. open new terminal and launch navigation programs
   1. `source devel/setup.bash`
   2. `roslaunch tb3_navigation move_base.launch`

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

