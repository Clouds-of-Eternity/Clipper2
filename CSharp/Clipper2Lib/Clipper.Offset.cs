/*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Date      :  11 October 2025                                                 *
* Website   :  https://www.angusj.com                                          *
* Copyright :  Angus Johnson 2010-2025                                         *
* Purpose   :  Path Offset (Inflate/Shrink)                                    *
* License   :  https://www.boost.org/LICENSE_1_0.txt                           *
*******************************************************************************/

#nullable enable
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using EternityWorks;

namespace Clipper2Lib
{
  public enum JoinType
  {
    Miter,
    Square,
    Bevel,
    Round
  }

  public enum EndType
  {
    Polygon,
    Joined,
    Butt,
    Square,
    Round
  }

  public class ClipperOffset
  {

    private class Group
    {
      internal PathsPoint inPaths;
      internal JoinType joinType;
      internal EndType endType;
      internal bool pathsReversed;
      internal int lowestPathIdx;

      public Group(PathsPoint paths, JoinType joinType, EndType endType = EndType.Polygon)
      {
        this.joinType = joinType;
        this.endType = endType;

        bool isJoined = ((endType == EndType.Polygon) || (endType == EndType.Joined));
        inPaths = new PathsPoint(paths.Count);
        foreach(PathPoint path in paths)
          inPaths.Add(Clipper.StripDuplicates(path, isJoined));

        if (endType == EndType.Polygon)
        {
          bool isNegArea;
          GetLowestPathInfo(inPaths, out lowestPathIdx, out isNegArea);
          // the lowermost path must be an outer path, so if its orientation is negative,
          // then flag that the whole group is 'reversed' (will negate delta etc.)
          // as this is much more efficient than reversing every path.
          pathsReversed = (lowestPathIdx >= 0) && isNegArea;
        }
        else
        {
          lowestPathIdx = -1;
          pathsReversed = false;
        }
      }
    }

    private const float Tolerance = 1.0E-12f;

    // Clipper2 approximates arcs by using series of relatively short straight
    //line segments. And logically, shorter line segments will produce better arc
    // approximations. But very short segments can degrade performance, usually
    // with little or no discernable improvement in curve quality. Very short
    // segments can even detract from curve quality, due to the effects of integer
    // rounding. Since there isn't an optimal number of line segments for any given
    // arc radius (that perfectly balances curve approximation with performance),
    // arc tolerance is user defined. Nevertheless, when the user doesn't define
    // an arc tolerance (ie leaves alone the 0 default value), the calculated
    // default arc tolerance (offset_radius / 500) generally produces good (smooth)
    // arc approximations without producing excessively small segment lengths.
    // See also: https://www.angusj.com/clipper2/Docs/Trigonometry.htm
    private const float arc_const = 0.002f; // <-- 1/500

    private readonly List<Group> _groupList = new List<Group>();
    private PathPoint pathOut = new PathPoint();
    private readonly PathVector2 _normals = new PathVector2();
    private PathsPoint _solution = new PathsPoint();
    private PolyTree64? _solutionTree;

    private float _groupDelta; //*0.5 for open paths; *-1.0 for negative areas
    private float _delta;
    private float _mitLimSqr;
    private float _stepsPerRad;
    private float _stepSin;
    private float _stepCos;
    private JoinType _joinType;
    private EndType _endType;
    public float ArcTolerance { get; set; }
    public bool MergeGroups { get; set; }
    public float MiterLimit { get; set; }
    public bool PreserveCollinear { get; set; }
    public bool ReverseSolution { get; set; }

    public delegate float DeltaCallback64(PathPoint path,
      PathVector2 path_norms, int currPt, int prevPt);
    public DeltaCallback64? DeltaCallback { get; set; }

    public ClipperOffset(float miterLimit = 2.0f,
      float arcTolerance = 0.0f, bool
      preserveCollinear = false, bool reverseSolution = false)
    {
      MiterLimit = miterLimit;
      ArcTolerance = arcTolerance;
      MergeGroups = true;
      PreserveCollinear = preserveCollinear;
      ReverseSolution = reverseSolution;
    }
    public void Clear()
    {
      _groupList.Clear();
    }

    public void AddPath(PathPoint path, JoinType joinType, EndType endType)
    {
      int cnt = path.Count;
      if (cnt == 0) return;
      PathsPoint pp = new PathsPoint(1) { path };
      AddPaths(pp, joinType, endType);
    }

    public void AddPaths(PathsPoint paths, JoinType joinType, EndType endType)
    {
      int cnt = paths.Count;
      if (cnt == 0) return;
      _groupList.Add(new Group(paths, joinType, endType));
    }

    private int CalcSolutionCapacity()
    {
      int result = 0;
      foreach (Group g in _groupList)
        result += (g.endType == EndType.Joined) ? g.inPaths.Count * 2 : g.inPaths.Count;
      return result;
    }

    internal bool CheckPathsReversed()
    {
      bool result = false;
      foreach (Group g in _groupList)
        if (g.endType == EndType.Polygon)
        {
          result = g.pathsReversed;
          break;
        }
      return result;
    }

    private void ExecuteInternal(float delta)
    {
      if (_groupList.Count == 0) return;
      _solution.EnsureCapacity(CalcSolutionCapacity());

      // make sure the offset delta is significant
      if (Math.Abs(delta) < 0.5)
      {
        foreach (Group group in _groupList)
          foreach (PathPoint path in group.inPaths)
            _solution.Add(path);
        return;
      }

      _delta = delta;
      _mitLimSqr = (MiterLimit <= 1 ?
        2.0f : 2.0f / Clipper.Sqr(MiterLimit));

      foreach (Group group in _groupList)
        DoGroupOffset(group);

      if (_groupList.Count == 0) return;

      bool pathsReversed = CheckPathsReversed();
      FillRule fillRule = pathsReversed ? FillRule.Negative : FillRule.Positive;

      // clean up self-intersections ...
      Clipper64 c = new Clipper64();
      c.PreserveCollinear = PreserveCollinear;
      c.ReverseSolution = ReverseSolution != pathsReversed;
      c.AddSubject(_solution);
      if (_solutionTree != null)
        c.Execute(ClipType.Union, fillRule, _solutionTree);
      else
        c.Execute(ClipType.Union, fillRule, _solution);

    }

    public void Execute(float delta, PathsPoint solution)
    {
      solution.Clear();
      _solution = solution;
      ExecuteInternal(delta);
    }

    public void Execute(float delta, PolyTree64 solutionTree)
    {
      solutionTree.Clear();
      _solutionTree = solutionTree;
      _solution.Clear();
      ExecuteInternal(delta);
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Vector2 GetUnitNormal(Point pt1, Point pt2)
    {
      float dx = (pt2.X - pt1.X);
      float dy = (pt2.Y - pt1.Y);
      if ((dx == 0) && (dy == 0)) return new Vector2();

      float f = 1.0f / float.Sqrt(dx * dx + dy * dy);
      dx *= f;
      dy *= f;

      return new Vector2(dy, -dx);
    }

    public void Execute(DeltaCallback64 deltaCallback, PathsPoint solution)
    {
      DeltaCallback = deltaCallback;
      Execute(1.0f, solution);
    }    
    
    internal static void GetLowestPathInfo(PathsPoint paths, out int idx, out bool isNegArea)
    {
      idx = -1;
      isNegArea = false;
      Point botPt = new Point(int.MaxValue, int.MinValue);
      for (int i = 0; i < paths.Count; ++i)
      {
        float a = float.MaxValue;
        foreach (Point pt in paths[i])
		    {
          if ((pt.Y < botPt.Y) ||
            ((pt.Y == botPt.Y) && (pt.X >= botPt.X))) continue;
          if (a == float.MaxValue)
          {
            a = Clipper.Area(paths[i]);
            if (a == 0) break; // invalid closed path so break from inner loop
            isNegArea = a < 0;
          }
          idx = i;
          botPt.X = pt.X;
          botPt.Y = pt.Y;
        }
      }
    }

  [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2 TranslatePoint(Vector2 pt, float dx, float dy)
    {
      return new Vector2(pt.X + dx, pt.Y + dy);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2 ReflectPoint(Vector2 pt, Vector2 pivot)
    {
      return new Vector2(pivot.X + (pivot.X - pt.X), pivot.Y + (pivot.Y - pt.Y));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool AlmostZero(float value, float epsilon = 0.001f)
    {
      return Math.Abs(value) < epsilon;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float Hypotenuse(float x, float y)
    {
      return float.Sqrt(float.Pow(x, 2) + float.Pow(y, 2));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2 NormalizeVector(Vector2 vec)
    {
	    float h = Hypotenuse(vec.X, vec.Y);
	    if (AlmostZero(h)) return new Vector2(0,0);
        float inverseHypot = 1 / h;
	    return new Vector2(vec.X * inverseHypot, vec.Y * inverseHypot);
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector2 GetAvgUnitVector(Vector2 vec1, Vector2 vec2)
    {
	    return NormalizeVector(new Vector2(vec1.X + vec2.X, vec1.Y + vec2.Y));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Point GetPerpendic(Point pt, Vector2 norm)
    {
      return new Point(pt.X + (int)(norm.X * _groupDelta),
        pt.Y + (int)(norm.Y * _groupDelta));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Vector2 GetPerpendicD(Point pt, Vector2 norm)
    {
      return new Vector2(pt.X + norm.X * _groupDelta,
        pt.Y + norm.Y * _groupDelta);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoBevel(PathPoint path, int j, int k)
    {
      Point pt1, pt2;
      if (j == k)
      {
        float absDelta = Math.Abs(_groupDelta);
        pt1 = new Point(
          path[j].X - (int)(absDelta * _normals[j].X),
          path[j].Y - (int)(absDelta * _normals[j].Y));
        pt2 = new Point(
          path[j].X + (int)(absDelta * _normals[j].X),
          path[j].Y + (int)(absDelta * _normals[j].Y));
      }
      else
      {
        pt1 = new Point(
          path[j].X + (int)(_groupDelta * _normals[k].X),
          path[j].Y + (int)(_groupDelta * _normals[k].Y));
        pt2 = new Point(
          path[j].X + (int)(_groupDelta * _normals[j].X),
          path[j].Y + (int)(_groupDelta * _normals[j].Y));
      }
      pathOut.Add(pt1);
      pathOut.Add(pt2);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoSquare(PathPoint path, int j, int k)
    {
      Vector2 vec;
      if (j == k)
      {
        vec = new Vector2(_normals[j].Y, -_normals[j].X);
      }
      else
      {
        vec = GetAvgUnitVector(
          new Vector2(-_normals[k].Y, _normals[k].X),
          new Vector2(_normals[j].Y, -_normals[j].X));
      }

      float absDelta = Math.Abs(_groupDelta);
      // now offset the original vertex delta units aint unit vector
      Vector2 ptQ = path[j].ToVector2();
      ptQ = TranslatePoint(ptQ, absDelta * vec.X, absDelta * vec.Y);

      // get perpendicular vertices
      Vector2 pt1 = TranslatePoint(ptQ, _groupDelta * vec.Y, _groupDelta * -vec.X);
      Vector2 pt2 = TranslatePoint(ptQ, _groupDelta * -vec.Y, _groupDelta * vec.X);
      // get 2 vertices aint one edge offset
      Vector2 pt3 = GetPerpendicD(path[k], _normals[k]);

      if (j == k)
      {
        Vector2 pt4 = new Vector2(
          pt3.X + vec.X * _groupDelta,
          pt3.Y + vec.Y * _groupDelta);
        InternalClipper.GetLineIntersectPt(pt1, pt2, pt3, pt4, out Vector2 pt);
        //get the second intersect point through reflecion
        pathOut.Add(ReflectPoint(pt, ptQ).ToPoint());
        pathOut.Add(pt.ToPoint());
      }
      else
      {
        Vector2 pt4 = GetPerpendicD(path[j], _normals[k]);
        InternalClipper.GetLineIntersectPt(pt1, pt2, pt3, pt4, out Vector2 pt);
        pathOut.Add(pt.ToPoint());
        //get the second intersect point through reflecion
        pathOut.Add(ReflectPoint(pt, ptQ).ToPoint());
      }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoMiter(PathPoint path, int j, int k, float cosA)
    {
      float q = _groupDelta / (cosA + 1);
      pathOut.Add(new Point(
          path[j].X + (int)((_normals[k].X + _normals[j].X) * q),
          path[j].Y + (int)((_normals[k].Y + _normals[j].Y) * q)));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoRound(PathPoint path, int j, int k, float angle)
    {
      if (DeltaCallback != null)
      {
        // when DeltaCallback is assigned, _groupDelta won't be constant,
        // so we'll need to do the following calculations for *every* vertex.
        float absDelta = Math.Abs(_groupDelta);
        float arcTol = ArcTolerance > 0.01 ? ArcTolerance : absDelta * arc_const;
        float stepsPer360 = MathF.PI / MathF.Acos(1 - arcTol / absDelta);
        _stepSin = float.Sin((2 * MathF.PI) / stepsPer360);
        _stepCos = float.Cos((2 * MathF.PI) / stepsPer360);
        if (_groupDelta < 0.0) _stepSin = -_stepSin;
        _stepsPerRad = stepsPer360 / (2 * MathF.PI);
      }

      Point pt = path[j];
      Vector2 offsetVec = new Vector2(_normals[k].X * _groupDelta, _normals[k].Y * _groupDelta);
      if (j == k) offsetVec *= -1;
      pathOut.Add(new Point(pt.X + (int)offsetVec.X, pt.Y + (int)offsetVec.Y));
      int steps = (int) Math.Ceiling(_stepsPerRad * Math.Abs(angle));
      for (int i = 1; i < steps; i++) // ie 1 less than steps
      {
        offsetVec = new Vector2(offsetVec.X * _stepCos - _stepSin * offsetVec.Y,
            offsetVec.X * _stepSin + offsetVec.Y * _stepCos);
        pathOut.Add(new Point(pt.X + (int)offsetVec.X, pt.Y + (int)offsetVec.Y));
      }
      pathOut.Add(GetPerpendic(pt, _normals[j]));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void BuildNormals(PathPoint path)
    {
      int cnt = path.Count;
      _normals.Clear();
      if (cnt == 0) return;
      _normals.EnsureCapacity(cnt);
      for (int i = 0; i < cnt - 1; i++)
        _normals.Add(GetUnitNormal(path[i], path[i + 1]));
      _normals.Add(GetUnitNormal(path[cnt - 1], path[0]));
    }

    private void OffsetPoint(Group group, PathPoint path, int j, ref int k)
    {
      if (path[j] == path[k]) { k = j; return; }

      // Let A = change in angle where edges join
      // A == 0: ie no change in angle (flat join)
      // A == PI: edges 'spike'
      // sin(A) < 0: right turning
      // cos(A) < 0: change in angle is more than 90 degree
      float sinA = InternalClipper.CrossProduct(_normals[j], _normals[k]);
      float cosA = InternalClipper.DotProduct(_normals[j], _normals[k]);
      if (sinA > 1.0) sinA = 1.0f;
      else if (sinA < -1.0) sinA = -1.0f;

      if (DeltaCallback != null)
      { 
        _groupDelta = DeltaCallback(path, _normals, j, k);
        if (group.pathsReversed) _groupDelta = -_groupDelta;
      }
      if (Math.Abs(_groupDelta) < Tolerance)
      {
        pathOut.Add(path[j]);
        return;
      }

      if (cosA > -0.999 && (sinA * _groupDelta < 0)) // test for concavity first (#593)
      {
        // is concave
        // by far the simplest way to construct concave joins, especially those joining very 
        // short segments, is to insert 3 points that produce negative regions. These regions 
        // will be removed later by the finishing union operation. This is also the best way 
        // to ensure that path reversals (ie over-shrunk paths) are removed.
        pathOut.Add(GetPerpendic(path[j], _normals[k]));
        pathOut.Add(path[j]); // (#405, #873, #916)
        pathOut.Add(GetPerpendic(path[j], _normals[j]));
      }
      else if ((cosA > 0.999) && (_joinType != JoinType.Round))
      {
        // almost straight - less than 2.5 degree (#424, #482, #526 & #724) 
        DoMiter(path, j, k, cosA);
      }
      else switch (_joinType)
      {
        // miter unless the angle is sufficiently acute to exceed ML
        case JoinType.Miter when cosA > _mitLimSqr - 1:
          DoMiter(path, j, k, cosA);
          break;
        case JoinType.Miter:
          DoSquare(path, j, k);
          break;
        case JoinType.Round:
          DoRound(path, j, k, MathF.Atan2(sinA, cosA));
          break;
        case JoinType.Bevel:
          DoBevel(path, j, k);
          break;
        default:
          DoSquare(path, j, k);
          break;
      }

      k = j;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void OffsetPolygon(Group group, PathPoint path)
    {
      pathOut = new PathPoint();
      int cnt = path.Count, prev = cnt - 1;
      for (int i = 0; i < cnt; i++)
        OffsetPoint(group, path, i, ref prev);
      _solution.Add(pathOut);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void OffsetOpenJoined(Group group, PathPoint path)
    {
      OffsetPolygon(group, path);
      path = Clipper.ReversePath(path);
      BuildNormals(path);
      OffsetPolygon(group, path);
    }

    private void OffsetOpenPath(Group group, PathPoint path)
    {
      pathOut = new PathPoint();
      int highI = path.Count - 1;

      if (DeltaCallback != null) 
        _groupDelta = DeltaCallback(path, _normals, 0, 0);

      // do the line start cap
      if (Math.Abs(_groupDelta) < Tolerance)
        pathOut.Add(path[0]);
      else
        switch (_endType)
        {
          case EndType.Butt:
            DoBevel(path, 0, 0);
            break;
          case EndType.Round:
            DoRound(path, 0, 0, MathF.PI);
            break;
          default:
            DoSquare(path, 0, 0);
            break;
        }

      // offset the left side going forward
      for (int i = 1, k = 0; i < highI; i++)
        OffsetPoint(group, path, i, ref k);

      // reverse normals ...
      for (int i = highI; i > 0; i--)
        _normals[i] = new Vector2(-_normals[i - 1].X, -_normals[i - 1].Y);
      _normals[0] = _normals[highI];

      if (DeltaCallback != null)
        _groupDelta = DeltaCallback(path, _normals, highI, highI);
      // do the line end cap
      if (Math.Abs(_groupDelta) < Tolerance)
        pathOut.Add(path[highI]);
      else
        switch (_endType)
        {
          case EndType.Butt:
            DoBevel(path, highI, highI);
            break;
          case EndType.Round:
            DoRound(path, highI, highI, MathF.PI);
            break;
          default:
            DoSquare(path, highI, highI);
            break;
        }

      // offset the left side going back
      for (int i = highI -1, k = highI; i > 0; i--)
        OffsetPoint(group, path, i, ref k);

      _solution.Add(pathOut);
    }

    private void DoGroupOffset(Group group)
    {
      if (group.endType == EndType.Polygon)
      {
        // a straight path (2 points) can now also be 'polygon' offset 
        // where the ends will be treated as (180 deg.) joins
        if (group.lowestPathIdx < 0) _delta = Math.Abs(_delta);
        _groupDelta = (group.pathsReversed) ? -_delta : _delta;
      }
      else
        _groupDelta = Math.Abs(_delta);

      float absDelta = Math.Abs(_groupDelta);

      _joinType = group.joinType;
      _endType = group.endType;

      if (group.joinType == JoinType.Round || group.endType == EndType.Round)
      {
        float arcTol = ArcTolerance > 0.01 ? ArcTolerance : absDelta * arc_const;
        float stepsPer360 = MathF.PI / MathF.Acos(1 - arcTol / absDelta);
        _stepSin = float.Sin((2 * MathF.PI) / stepsPer360);
        _stepCos = float.Cos((2 * MathF.PI) / stepsPer360);
        if (_groupDelta < 0.0) _stepSin = -_stepSin;
        _stepsPerRad = stepsPer360 / (2 * MathF.PI);
      }

      using List<PathPoint>.Enumerator pathIt = group.inPaths.GetEnumerator();
      while (pathIt.MoveNext())
      {
        PathPoint p = pathIt.Current!;

        pathOut = new PathPoint();
        int cnt = p.Count;

        switch (cnt)
        {
          case 1:
          {
            Point pt = p[0];

            if (DeltaCallback != null)
            {
              _groupDelta = DeltaCallback(p, _normals, 0, 0);
              if (group.pathsReversed) _groupDelta = -_groupDelta;
              absDelta = Math.Abs(_groupDelta);
            }

            // single vertex so build a circle or square ...
            if (group.endType == EndType.Round)
            {
              int steps = (int) Math.Ceiling(_stepsPerRad * 2 * Math.PI);
              pathOut = Clipper.Ellipse(pt, absDelta, absDelta, steps);
            }
            else
            {
              int d = (int) Math.Ceiling(_groupDelta);
              Rectangle r = new Rectangle(pt.X - d, pt.Y - d, pt.X + d, pt.Y + d);
              pathOut = Clipper.AsPath(r);
            }
            _solution.Add(pathOut);
            continue; // end of offsetting a single point 
          }
          case 2 when group.endType == EndType.Joined:
            _endType = (group.joinType == JoinType.Round) ?
              EndType.Round :
              EndType.Square;
            break;
        }


        BuildNormals(p);
        switch (_endType)
        {
          case EndType.Polygon:
            OffsetPolygon(group, p);
            break;
          case EndType.Joined:
            OffsetOpenJoined(group, p);
            break;
          default:
            OffsetOpenPath(group, p);
            break;
        }
      }
    }
  }

} // namespace