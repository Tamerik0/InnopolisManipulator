using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Manipulator : MonoBehaviour
{
    public struct State
    {
        public float rotation1;
        public float rotation2;
        public float length;
        public float payloadSize;
        public float payloadRotation;
        public State(float r1, float r2, float l) : this(r1, r2, l, 0, 0)
        {
            normalize();
        }
        public State(float r1, float r2, float l, float payloadSize, float payloadRotation)
        {
            rotation1 = r1;
            rotation2 = r2;
            length = l;
            this.payloadSize = payloadSize;
            this.payloadRotation = payloadRotation;
        }
        public void normalize()
        {
            rotation1 = normalize180(rotation1);
            rotation2 = normalize180(rotation2);
        }
        public State normalized()
        {
            var ans = this;
            ans.normalize();
            return ans;
        }
    }
    public static float normalize180(float angle)
    {
        angle = Mathf.Repeat(angle, 360);
        if (angle > 180)
            return angle - 360;
        return angle;
    }

    public static float distanceBetweenAngles(float angle1, float angle2, int dir)
    {
        if (dir == 1)
        {
            return Mathf.Repeat(angle2 - angle1, 360);
        }
        else if (dir == -1)
        {
            return Mathf.Repeat(360 - Mathf.Repeat(angle2 - angle1, 360), 360);
        }
        throw new NotImplementedException();
    }
    public static int shortestPathBetweenAngles(float angle1, float angle2)
    {
        return distanceBetweenAngles(angle1, angle2, 1) < distanceBetweenAngles(angle1, angle2, -1) ? 1 : -1;
    }
    public static float shortestSignedDistanceBetweenAngles(float angle1, float angle2)
    {
        var d = shortestPathBetweenAngles(angle1, angle2);
        return d * distanceBetweenAngles(angle1, angle2, d);
    }
    [SerializeField] float l1;
    [SerializeField] float r;
    [SerializeField] float minL;
    [SerializeField] float maxL;
    [SerializeField] float rotationSpeed;
    [SerializeField] float lengthSpeed;
    public Vector3 targetPosition;
    [SerializeField] HingeJoint joint1;
    [SerializeField] HingeJoint joint2;
    [SerializeField] ControllableLengthLink link;
    [SerializeField] Electromagnet electromagnet;
    [SerializeField] Transform CoordinateSystem;
    private GameObject payload { get => electromagnet.payload == null ? null : electromagnet.payload.gameObject; set { electromagnet.payload = value == null ? null : value.GetComponent<Rigidbody>(); } }
    private List<State> currentPath;
    private int pathPoint;
    private Vector3 lastTargetPosition;
    private GameObject targetCube;
    private Vector3 cubePlace = new Vector3(0, 0.05f, 0);
    public State state
    {
        get
        {
            if (payload == null)
                return new State(joint1.spring.targetPosition, joint2.spring.targetPosition, link.Length);
            else
                return new State(joint1.spring.targetPosition, joint2.spring.targetPosition, link.Length, payload.transform.localScale.x, payload.transform.rotation.eulerAngles.y);
        }
        set
        {
            var spring1 = joint1.spring;
            spring1.targetPosition = value.rotation1;
            joint1.spring = spring1;
            var spring2 = joint2.spring;
            spring2.targetPosition = value.rotation2;
            joint2.spring = spring2;
            link.Length = value.length;
        }
    }
    public bool checkState(State state)
    {
        if (state.length < minL || state.length > maxL)
        {
            return false;
        }
        if (l1 + state.length * Mathf.Cos(state.rotation2 * Mathf.Deg2Rad) * state.length - Mathf.Max(r, state.payloadSize / 2 * Mathf.Sqrt(2)) < 0)
        {
            return false;
        }
        if (Mathf.Repeat(state.rotation2, 360) > 180)
        {
            if (r * Mathf.Tan(Mathf.Abs(Mathf.PI / 2 + state.rotation2 * Mathf.Deg2Rad)) < state.payloadSize / 2 * Mathf.Sqrt(2))
            {
                return false;
            }
        }
        var r1 = state.rotation1;
        var r2 = state.rotation2;
        Vector3 p1 = CoordinateSystem.TransformPoint(new Vector3(0, l1, 0));
        Vector3 p2 = CoordinateSystem.TransformPoint(Quaternion.Euler(0, -r1, 0) * new Vector3(l1, l1, 0));
        Vector3 p3 = CoordinateSystem.TransformPoint(Quaternion.Euler(0, -r1, 0) * (new Vector3(l1, l1, 0) + Quaternion.Euler(0, 0, -r2) * new Vector3(state.length, 0, 0)));
        var intersections = Physics.OverlapBox((p1 + p2) / 2, new Vector3(0.05f, 0.05f, l1 / 2), Quaternion.LookRotation(p2 - p1))
            .Concat(Physics.OverlapBox((p2 + p3) / 2, new Vector3(0.1f, 0.1f, state.length / 2), Quaternion.LookRotation(p3 - p2)))
            .Concat(Physics.OverlapSphere(p3, r))
            .Concat(Physics.OverlapBox(p3 + Vector3.down * (r + (state.payloadSize + (state.payloadSize!=0? 0.05f:0)) / 2), Vector3.one * (state.payloadSize + (state.payloadSize != 0 ? 0.05f : 0)) / 2, Quaternion.Euler(0, state.payloadRotation, 0))).ToList();
        foreach (var collider in intersections)
        {
            if (collider.gameObject.layer != 3 && collider.gameObject != payload)
            {
                return false;
            }
        }
        return true;
    }
    public List<State> CalculatePath(State beginState, State endState, float deltaAngle, int dirR1 = 0)
    {
        if (dirR1 == 0)
        {
            if (!checkState(endState))
                return null;
            int d = Mathf.Repeat(endState.rotation1 - beginState.rotation1, 360) < 180 ? 1 : -1;
            var ans = CalculatePath(beginState, endState, deltaAngle, d);
            if (ans != null)
                return ans;
            else
                return CalculatePath(beginState, endState, deltaAngle, -d);
        }
        else
        {
            var t = Mathf.Repeat(Mathf.Abs(endState.rotation1 - beginState.rotation1), 360);
            if (Mathf.Min(t, 360 - t) <= deltaAngle / 2)
            {
                for (int d1 = -1; d1 <= 1; d1 += 2)
                {
                    for (float a = beginState.rotation2; (a - beginState.rotation2) * d1 < 360; a += d1 * deltaAngle)
                    {
                        if (!checkState(new State(endState.rotation1, a, minL, beginState.payloadSize, beginState.payloadRotation)))
                            break;
                        if (Mathf.Abs(shortestSignedDistanceBetweenAngles(a, endState.rotation2)) <= deltaAngle / 2)
                        {
                            if (a != beginState.rotation2)
                                return new List<State>() { endState, new State(endState.rotation1, endState.rotation2, minL), new State(endState.rotation1, beginState.rotation2 + d1 * distanceBetweenAngles(beginState.rotation2, endState.rotation2, d1) / 2, minL), new State(beginState.rotation1, beginState.rotation2, minL), beginState };
                            else
                                return new List<State>() { endState, beginState };
                        }
                    }

                }
                return null;
            }
            float nextR1 = beginState.rotation1 + dirR1 * deltaAngle;
            var list = new HashSet<State>();
            for (int d1 = -1; d1 <= 2; d1 += 2)
            {
                bool b = true;
                for (float a = beginState.rotation2; (a - beginState.rotation2) * d1 < 360; a += d1 * deltaAngle)
                {
                    if (checkState(new State(beginState.rotation1, a, minL, beginState.payloadSize, beginState.payloadRotation)))
                    {
                        var s = new State(nextR1, a, minL, beginState.payloadSize, beginState.payloadRotation);
                        if (checkState(s))
                        {
                            if (b)
                            {
                                b = false;
                                if (!list.Contains(s))
                                {
                                    list.Add(s);
                                    var path = CalculatePath(s, endState, deltaAngle, dirR1);
                                    if (path != null)
                                    {
                                        path.Add(new State(beginState.rotation1, s.rotation2, minL, beginState.payloadSize, beginState.payloadRotation));
                                        var d = distanceBetweenAngles(beginState.rotation2, s.rotation2, d1);
                                        if (d >= 180)
                                        {
                                            path.Add(new State(beginState.rotation1, s.rotation2 + d1 * d / 2, minL, beginState.payloadSize, beginState.payloadRotation));
                                        }
                                        path.Add(beginState);
                                        return path;
                                    }
                                }
                            }
                        }
                        else
                            b = true;
                    }
                    else break;
                }
            }
        }
        return null;
    }
    void CalculatePath()
    {
        var p = CoordinateSystem.InverseTransformPoint(targetPosition);
        var x = p.x;
        var y = p.y;
        var z = p.z;
        var l = Mathf.Sqrt(2 * l1 * l1 + x * x + y * y + z * z - 2 * l1 * (Mathf.Sqrt(x * x + z * z) + y));
        var targetState = new State(Mathf.Atan2(z, x) / Mathf.PI * 180, Mathf.Atan2(l1 - y, Mathf.Sqrt(x * x + z * z) - l1) / Mathf.PI * 180, l);
        currentPath = CalculatePath(state, targetState, 1);
        if (currentPath != null)
        {
            pathPoint = currentPath.Count - 1;
        }
    }
    void FixedUpdate()
    {
        if (payload == null && targetCube == null)
        {
            var cubes = GameObject.FindGameObjectsWithTag("a").ToList();
            cubes.Sort((GameObject a, GameObject b) => a.transform.localScale.magnitude < b.transform.localScale.magnitude ? 1 : a.transform.localScale.magnitude == b.transform.localScale.magnitude ? 0 : -1);
            foreach (var cube in cubes)
            {
                targetPosition = cube.transform.position + (cube.transform.localScale.y / 2 + r + 0.03f) * Vector3.up;
                CalculatePath();
                if (currentPath != null)
                {
                    targetCube = cube;
                    lastTargetPosition = targetPosition;
                    break;
                }
            }
        }
        if (payload == null)
        {
            if ((electromagnet.transform.position - targetPosition).magnitude < 0.02)
            {
                payload = targetCube;
                targetPosition = cubePlace + (targetCube.transform.localScale.y + r) * Vector3.up;
                targetCube = null;
            }
        }
        if (payload != null)
        {
            if ((payload.transform.position + payload.transform.localScale.y / 2 * Vector3.down - cubePlace).magnitude < 0.1)
            {
                payload.tag = "Untagged";
                cubePlace += Vector3.up * payload.transform.localScale.y;
                payload = null;
            }
        }
        if (lastTargetPosition != targetPosition)
        {
            CalculatePath();
            lastTargetPosition = targetPosition;
        }
        var t = Mathf.Min(Time.deltaTime, 2 * Time.fixedDeltaTime);
        Vector3 dir;
        while (true)
        {
            var target = currentPath[pathPoint];
            target.normalize();
            dir = new Vector3(shortestSignedDistanceBetweenAngles(state.rotation1, target.rotation1) / rotationSpeed, shortestSignedDistanceBetweenAngles(state.rotation2, target.rotation2) / rotationSpeed, (target.length - state.length) / lengthSpeed);
            var m = Mathf.Max(Mathf.Abs(dir.x), Mathf.Abs(dir.y), Mathf.Abs(dir.z));
            if (m == 0)
            {
                dir = Vector3.zero;
            }
            else
            {
                dir /= m;
            }
            if (m <= t)
            {
                state = currentPath[pathPoint].normalized();
                t -= m;
                if (pathPoint != 0)
                {
                    pathPoint--;
                }
                else
                {
                    t = 0;
                    break;
                }
            }
            else
            {
                break;
            }
        }
        dir *= t;
        var dr1 = dir.x * rotationSpeed;
        var dr2 = dir.y * rotationSpeed;
        var dl = dir.z * lengthSpeed;
        var s = state;
        s.rotation1 += dr1;
        s.rotation2 += dr2;
        s.length += dl;
        s.normalize();
        if (checkState(s))
            state = s;
        else
        {
            if (Mathf.Abs(state.rotation1 - currentPath[pathPoint].rotation1) < 0.1f && Mathf.Abs(state.length - currentPath[pathPoint].length) < 0.1)
            {
                pathPoint--;
            }
            else
            {
                s = state;
                s.rotation2 -= rotationSpeed * Time.deltaTime;
                if (checkState(s))
                {
                    s.normalize();
                    state = s;
                }
                else
                {
                    s.length -= lengthSpeed * Time.deltaTime;
                    if (checkState(s))
                    {
                        s.normalize();
                        state = s;
                    }
                    else
                    {
                        s.length += 2 * lengthSpeed * Time.deltaTime;
                        if (checkState(s))
                        {
                            s.normalize();
                            state = s;
                        }
                        else
                            print("Can't move to target position");
                    }
                }
            }
        }
    }
}
