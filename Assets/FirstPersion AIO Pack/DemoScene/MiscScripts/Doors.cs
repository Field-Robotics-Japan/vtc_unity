using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Doors : MonoBehaviour {

void OnTriggerEnter(Collider coll){
 if(coll.tag=="Player"){
	 GetComponent<Animator>().Play("Door_open");
	 this.enabled=false;
 }
}
}
