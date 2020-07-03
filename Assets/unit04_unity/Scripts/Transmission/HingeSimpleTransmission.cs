using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HingeSimpleTransmission : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        var hinge = GetComponent<HingeJoint>();

        // Make the hinge motor rotate with 90 degrees per second and a strong force.
        var motor = hinge.motor;
        // motor.force = 1000;
        motor.targetVelocity = 0;
        motor.freeSpin = false;
        hinge.motor = motor;
        hinge.useMotor = true;
    }

    // Update is called once per frame
    void Update()
    {

    }
}
