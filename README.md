http_bridge
=======

概要
=======
このパッケージは東日本計算センター様にて開発された[RDR](https://rtc-fukushima.jp/5th-robot-symposium-aizu/5th-robot-symposium-aizu-03/)とのHTTPプロトコルを介した通信を行うためのブリッジモジュールです。
将来的にはpythonへ移行し、rosbridge_libraryによる汎用的なブリッジモジュール化を目指します。

インストール方法
=======
### 1．ROSワークスペースのディレクトリに移動し、リポジトリをクローン
```bash 
cd ~/{ROSワークスペースディレクトリ}/src/
git clone https://github.com/jadsys/http_bridge.git
```
### 2．Buildを行う
```bash 
cd http_bridge
catkin build --this
```
### X. 依存関係の解決
当パッケージでは外部パッケージとして以下を利用しております。
- [C++ Requests: Curl for People](https://github.com/libcpr/cpr)
- [JSON for Modern C++](https://github.com/nlohmann/json)
- [uoa_poc3_msgs](https://github.com/jadsys/uoa_poc3_msgs.git)
- [uoa_poc4_msgs](https://github.com/jadsys/uoa_poc4_msgs.git)
- [uoa_poc5_msgs](https://github.com/jadsys/uoa_poc5_msgs.git)
- [uoa_poc6_msgs](https://github.com/jadsys/uoa_poc6_msgs.git)

それぞれのインストールを行います。

HTTP通信用ライブラリ"C++ Requests: Curl for People"については、以下のコマンドを入力しインストールします。
```bash
mkdir ~/work && cd ~/work/
git clone -b "2023年度成果物" https://github.com/libcpr/cpr.git
cd cpr && mkdir build && cd build
cmake .. -DCPR_USE_SYSTEM_CURL=ON
cmake --build .
sudo cmake --install .
```

その他のパッケージは以下の方法でインストール可能です。
```bash
# vcsツールのインストール（既にインストール済みの場合スキップ）
sudo pip install -U vcstool

# rosdepのインストール
sudo apt install python3-rosdep
sudo rosdep init # 過去に実行済みの場合は実行不要
rosdep update # 過去に実行済みの場合は実行不要

# 依存関係のインストール
cd ~/{ROSワークスペースディレクトリ}/src
vcs import  < http_bridge/dependency.rosinstall
rosdep install -i --from-paths http_bridge
catkin build
```

ライセンス
=======
## BSD 3-Clause License

Copyright (c) 2023, Japan Advanced System,Ltd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors 
   may be used to endorse or promote products derived from this software 
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

* * *
## 使用ライブラリ関係
### [JSON for Modern C++](https://github.com/nlohmann/json)
 [MIT License](https://opensource.org/licenses/MIT): Copyright &copy; 2013-2022 [Niels Lohmann](https://nlohmann.me)
### [C++ Requests: Curl for People](https://github.com/libcpr/cpr)
 [MIT License](https://opensource.org/licenses/MIT): Copyright (c) 2017-2021 Huu Nguyen
 [MIT License](https://opensource.org/licenses/MIT): Copyright (c) 2022 libcpr and many other contributors