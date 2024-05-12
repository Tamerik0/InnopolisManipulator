using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Electromagnet : MonoBehaviour
{
    private Rigidbody _payload = null;
    public Rigidbody payload
    {
        get { return _payload; }
        set
        {
            if (_payload != null)
            {
                _payload.constraints = RigidbodyConstraints.None;
            }
            if (value != null)
            {
                value.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationY | RigidbodyConstraints.FreezeRotationZ;
            }
            _payload = value;
        }
    }
    void FixedUpdate()
    {
        if (payload != null)
        {
            payload.AddForce((transform.position - payload.position).normalized * 100);
        }
    }
}
