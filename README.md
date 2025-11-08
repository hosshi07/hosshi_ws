# hosshi_ws
ROS2のパッケージについて記載．

## ナビゲーションの便利化

### ロケーションの追加
ナビゲーションのマップをもとにしてロボットの現在地をロケーションとして登録できる.
これにより`go_to_location`のによる場所移動ができる.  

```bash
ros2 run happy_mobility location_get
```

サービス通信である  

| Name | Mssage | Request | Response |
| --- | --- | --- | --- |
| /save_pose | [Location] | string: `map` string: `location`  | bool: `success` |

`map`にはロケーションを登録したいマップ（pgm, yaml）を送ること  
`location`にはその場所の名前を入力してね.  

### ロケーションの場所へ移動.

`location_get`によって登録したロケーションまでナビゲーションを行う.  

 絶対に使えるようにする．  


| Name | Mssage | Request | Response |
| --- | --- | --- | --- |
| /go_to_location | [GoToLocation] | string: `location`  | bool: `success` |

