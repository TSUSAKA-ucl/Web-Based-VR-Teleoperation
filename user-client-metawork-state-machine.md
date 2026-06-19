# UserClient(WebXR)==browser==Quest1のmetaworkプロトコル用のstate machine

1. 状態
   * init: DOMマウント `mgr/register`する前
   * registered: `mgr/register`された。`mgr/request`できる
   * allowed: `mgr/request`が許可され`dev/user-id`を受信した(`robot-id`判明)
   * ready: `robot/robot-id`受信して、`joint-move-to`でVR画像が移動中(mqttのpublishはしない)
   * run: mqtt publish(`control/user-id`または`control/robot-id/user-id`)する。
	 `robot/robot-id`のジョイントとの偏差が大きすぎる場合は readyに遷移
   * finish: `mgr/unregister`publish, DOMアンマウント

2. (遷移)入力 & 動作
   * dormant -> init : --- &init pub `mgr/register`
   * init -> registered : なし(本当は多分register完了ack必要) &registered pub `mgr/request`
   * registered -> allowed : recv `dev/user-id` &allowed sub `robot/robot-id`(`/dev/user-id`ペイロード解析)
   * ここにselectingが入ったほうが良いか? 
   * allowed -> ready : recv `robot/robot-id` &ready `setAttribute('joint-move-to',)` or 直接 setJointTarget
   * ready -> run : event `ik-worker-arrival` remove attribute  &pub `control/user-id`
   * run -> ready : difference too large &ready
   * run -> registered : recv `??? release` unregistered &
   * ready -> registered
   * run -> finish
   * ready -> finish


robot IDの考え方をどのようにするか。UserClientはどのようにして個々のrobotを選択するか、あるいは選択でなくManagerの指示に基づき使用するか。Managerが指示する場合は、requestというより、操作可能なrobot typeの一覧およびそのプロパティー(右手? 左手?)

robotハンドとの構成をどのように考えるか。ハンドを含めて複数のa-entityからなる(attachしたもの)グループを定義して、それらのジョイント値を集めてpublishする?

ブラウザ上のmetawork communicatorコンポーネントはrobotとは独立に定義して、複数の個々のロボットのDOM idを保持するか。その場合、typeと実robotが保持するrobotIDは? 矛盾がないかどうかの検証は? httpサーバーがrobot-loaderを利用して実ロボットと同じ構成のロボットを組み立ててブラウザに渡したほうが良いのでは? 
