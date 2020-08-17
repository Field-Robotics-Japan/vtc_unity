using UnityEngine;

public class DoorController : MonoBehaviour
{
    // Inspector上にプルダウンメニュー追加
    public enum DoorType
    {
        NormalDoor,
        SlideDoor
    }
    [SerializeField] DoorType doorType;

    [SerializeField] private GameObject sensor;
    [SerializeField] private float RotateTarget;
    [SerializeField] private float SlideTarget;

    // 初期値（正規化用）
    private Vector3 initialJointPosition;
    private Vector3 initialJointRotate;

    private void Start()
    {
        sensor.GetComponent<DoorSensor>().isOpen = false;
        initialJointPosition   = this.gameObject.transform.localPosition;
        initialJointRotate     = this.gameObject.transform.localEulerAngles;
    }

    void Update()
    {
        switch (doorType)
        {
            // switchの分岐に文字列は使えない。
            // enumのリストを実装しておくとインデックスで分岐ができる。
            case DoorType.NormalDoor:
                if (sensor.GetComponent<DoorSensor>().isOpen)  DoorRotate(RotateTarget);    // 引数は目標角
                if (!sensor.GetComponent<DoorSensor>().isOpen) DoorRotate(0);
                return;
            case DoorType.SlideDoor:
                if (sensor.GetComponent<DoorSensor>().isOpen) DoorSlide(SlideTarget);      // 引数は目標移動量
                if (!sensor.GetComponent<DoorSensor>().isOpen) DoorSlide(0);
                return;
            default:
                return;
        }
    }

    private void DoorRotate(float targetAngle)
    {
        float nowAngle = this.transform.localEulerAngles.y;
        if (nowAngle > 180) nowAngle -= 360;
        float speed = (targetAngle - (nowAngle - initialJointRotate.y)) * 0.01f;    // P制御
        this.transform.Rotate(Vector3.up * speed);
    }

    private void DoorSlide(float targetPosition)
    {
        float speed = (targetPosition + (this.transform.localPosition.x - initialJointPosition.x)) * 0.01f;  // P制御
        this.transform.Translate(Vector3.left * speed);
    }
}
