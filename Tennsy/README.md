#### 一、安装环境
> sudo apt-get update  
sudo apt-get install python-gudev  
sudo apt-get install -y avahi-daemon  
sudo apt-get install -y openssh-server  
sudo apt-get install -y arduino-core  
sudo easy_install pip  
sudo pip install -U platformio  
sudo rm -rf ~/.platformio/  
wget https://www.pjrc.com/teensy/49-teensy.rules  
sudo cp 49-teensy.rules /etc/udev/rules.d/


#### 二、编译已有的工程
第一次编译可能需要久等一下，它需要安装相关平台的环境，这里以两驱动的为示例(具体代码路径，以提供资料为准)，请将两驱的源代码放入到提供虚拟机文件ubuntu 16.04环境中任意目录（或者用户通过上面的方法自己构建的环境也可以）。
> $ cd rikirobot_2wdarduino  
$ platformio run

#### 三、烧写固件到硬件里面
注意teensy 烧写会要求按一下板上的那个小按钮，编译出现“hint: press the reset button”，在按按键
> $ cd rikirobot_2wdarduino  
$ platformio run --target upload  
此命令会编译上传相关固件

#### 四、创建一个新的工作开发环境
> $mkdir test  
$ cd test  
$ platformio init --board teensy31

上面的工具也可以建立其它的平台如果arduino uno stm32有兴趣的可以自己去研究，另外关于此工具的IDE开发是ATOM + platformio，我有时间在出个教程，网上有安装教程，有兴趣的可以自己研究装好，比原装的Arduino IDE 好用很多，至少有函数跳转，语法高亮，你就懂了
