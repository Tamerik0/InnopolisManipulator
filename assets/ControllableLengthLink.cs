using UnityEngine;
public class ControllableLengthLink : MonoBehaviour
{
    [SerializeField]
    [Range(0f, 10f)]
    private float length;
    public float Length
    {
        get => length; set
        {
            length = value;
            var scale = transform.localScale;
            scale.y = Length / 2;
            transform.localScale = scale;
            joint.connectedAnchor = new Vector3(0, -1, 0);
            joint1.anchor = new Vector3(0, 1, 0);
        }
    }
    [SerializeField] HingeJoint joint;
    private Joint joint1;
    void Start()
    {
        joint1 = GetComponent<Joint>();
        Length = length;
    }
    void Update()
    {
        Length = length;
    }

}
