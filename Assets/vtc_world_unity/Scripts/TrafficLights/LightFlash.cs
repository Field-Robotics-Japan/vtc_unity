using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Experimental.Rendering.Universal;
using UnityEngine.UI;


public class LightFlash : MonoBehaviour
{
    [SerializeField] AnimationCurve curve;
    [SerializeField] float speed;
    [SerializeField] float intensity;

    Light light;
    float t = 0f;

    // Start is called before the first frame update
    void Start()
    {
        light = GetComponent<Light>();
    }

    // Update is called once per frame
    void Update()
    {
        t += Time.deltaTime;

        light.intensity = intensity * curve.Evaluate(t * speed);
    }
}
