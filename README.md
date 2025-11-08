# hosshi_ws
ROS2のパッケージについて記載．

## ナビゲーションの便利化

### ロケーションの追加
ナビゲーションのマップをもとにしてロボットの現在地をロケーションとして登録できる.
これにより`go_to_location`のによる場所移動ができる.  

```bash
ros2 run happy_mobility location_get #現段階
```

サービス通信である  

| Name | Mssage | Request | Response |
| --- | --- | --- | --- |
| /save_pose | [Location]() | string: `location`  | bool: `success` |

`map`にはロケーションを登録したいマップ（pgm, yaml）を送ること  
`location`にはその場所の名前を入力してね.  

### ロケーションの確認


| Name | Mssage | Request | Response |
| --- | --- | --- | --- |
| /location_list | [GetLocation]() | なし  | string[]: `location_list` |

登録したロケーションがリストで返ってくる  

### ロケーションの削除


| Name | Mssage | Request | Response |
| --- | --- | --- | --- |
| /remove_pose | [Location]() | string: `location`  | bool: `success` |

指定したロケーションを消す．


### ロケーションの場所へ移動.

`location_get`によって登録したロケーションまでナビゲーションを行う.  

 絶対に使えるようにする．  


| Name | Mssage | Request | Response |
| --- | --- | --- | --- |
| /go_to_location | [GoToLocation]() | string: `location`  | bool: `success` |



## 追加したいもの

`location_get`に

---

/save_pose サービスが完了したら /ros2 topic echo /amcl_pose の値も自動で出力

/list_locations サービスで登録済み目的地を一覧表示

/delete_location サービスで不要な目的地削除

---

これらのサービスを追加したい．  
**完了した**

