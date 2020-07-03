using UnityEngine;
using System.Collections;

public class ObjectRotation : MonoBehaviour {
	
	public float X;
	public float Y;
	public float Z;

	void Update () {
		transform.Rotate (new Vector3 (X, Y, Z) * Time.deltaTime );

	}
}
