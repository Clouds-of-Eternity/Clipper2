/*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Date      :  14 December 2025                                                *
* Website   :  https://www.angusj.com                                          *
* Copyright :  Angus Johnson 2010-2025                                         *
* Purpose   :  This module contains simple functions that will likely cover    *
*              most polygon boolean and offsetting needs, while also avoiding  *
*              the inherent complexities of the other modules.                 *
* Thanks    :  Special thanks to Thong Nguyen, Guus Kuiper, Phil Stopford,     *
*           :  and Daniel Gosnell for their invaluable assistance with C#.     *
* License   :  https://www.boost.org/LICENSE_1_0.txt                           *
*******************************************************************************/

#nullable enable
using System;
using System.Numerics;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using EternityWorks;

namespace Clipper2Lib
{
  public static class Clipper
  {
    private static Rectangle invalidRectangle = new(int.MaxValue, int.MaxValue, int.MinValue, int.MinValue);
    public static Rectangle InvalidRectangle => invalidRectangle;

    private static Box invalidBox = new(float.MaxValue, float.MaxValue, float.MinValue, float.MinValue);
    public static Box InvalidBox => invalidBox;

    public static PathsPoint Intersect(PathsPoint subject, PathsPoint clip, FillRule fillRule)
    {
      return BooleanOp(ClipType.Intersection, subject, clip, fillRule);
    }

    public static PathsVector2 Intersect(PathsVector2 subject, PathsVector2 clip, 
      FillRule fillRule, int precision = 2)
    {
      return BooleanOp(ClipType.Intersection,
        subject, clip, fillRule, precision);
    }

    public static PathsPoint Union(PathsPoint subject, FillRule fillRule)
    {
      return BooleanOp(ClipType.Union, subject, null, fillRule);
    }

    public static PathsPoint Union(PathsPoint subject, PathsPoint clip, FillRule fillRule)
    {
      return BooleanOp(ClipType.Union, subject, clip, fillRule);
    }

    public static PathsVector2 Union(PathsVector2 subject, FillRule fillRule)
    {
      return BooleanOp(ClipType.Union, subject, null, fillRule);
    }

    public static PathsVector2 Union(PathsVector2 subject, PathsVector2 clip, 
      FillRule fillRule, int precision = 2)
    {
      return BooleanOp(ClipType.Union,
        subject, clip, fillRule, precision);
    }

    public static PathsPoint Difference(PathsPoint subject, PathsPoint clip, FillRule fillRule)
    {
      return BooleanOp(ClipType.Difference, subject, clip, fillRule);
    }

    public static PathsVector2 Difference(PathsVector2 subject, PathsVector2 clip, 
      FillRule fillRule, int precision = 2)
    {
      return BooleanOp(ClipType.Difference,
        subject, clip, fillRule, precision);
    }

    public static PathsPoint Xor(PathsPoint subject, PathsPoint clip, FillRule fillRule)
    {
      return BooleanOp(ClipType.Xor, subject, clip, fillRule);
    }

    public static PathsVector2 Xor(PathsVector2 subject, PathsVector2 clip, 
      FillRule fillRule, int precision = 2)
    {
      return BooleanOp(ClipType.Xor, 
        subject, clip, fillRule, precision);
    }

    public static PathsPoint BooleanOp(ClipType clipType,
      PathsPoint? subject, PathsPoint? clip, FillRule fillRule)
    {
      PathsPoint solution = new PathsPoint();
      if (subject == null) return solution;
      Clipper64 c = new Clipper64();
      c.AddPaths(subject, PathType.Subject);
      if (clip != null)
        c.AddPaths(clip, PathType.Clip);
      c.Execute(clipType, fillRule, solution);
      return solution;
    }

    public static void BooleanOp(ClipType clipType,
      PathsPoint? subject, PathsPoint? clip, 
      PolyTree64 polytree, FillRule fillRule)
    {
      if (subject == null) return;
      Clipper64 c = new Clipper64();
      c.AddPaths(subject, PathType.Subject);
      if (clip != null)
        c.AddPaths(clip, PathType.Clip);
      c.Execute(clipType, fillRule, polytree);
    }

    public static PathsVector2 BooleanOp(ClipType clipType, PathsVector2 subject, PathsVector2? clip, 
      FillRule fillRule, int precision = 2)
    {
      PathsVector2 solution = new PathsVector2();
      ClipperD c = new ClipperD(precision);
      c.AddSubject(subject);
      if (clip != null)
        c.AddClip(clip);
      c.Execute(clipType, fillRule, solution);
      return solution;
    }

    public static void BooleanOp(ClipType clipType,
      PathsVector2? subject, PathsVector2? clip,
      PolyTreeD polytree, FillRule fillRule, int precision = 2)
    {
      if (subject == null) return;
      ClipperD c = new ClipperD(precision);
      c.AddPaths(subject, PathType.Subject);
      if (clip != null)
        c.AddPaths(clip, PathType.Clip);
      c.Execute(clipType, fillRule, polytree);
    }

    public static PathsPoint InflatePaths(PathsPoint paths, float delta, JoinType joinType,
      EndType endType, float miterLimit = 2.0f, float arcTolerance = 0.0f)
    {
      ClipperOffset co = new ClipperOffset(miterLimit, arcTolerance);
      co.AddPaths(paths, joinType, endType);
      PathsPoint solution = new PathsPoint();
      co.Execute(delta, solution);
      return solution;
    }

    public static PathsVector2 InflatePaths(PathsVector2 paths, float delta, JoinType joinType,
      EndType endType, float miterLimit = 2.0f, int precision = 2, float arcTolerance = 0.0f)
    {
      InternalClipper.CheckPrecision(precision);
      float scale = float.Pow(10, precision);
      PathsPoint tmp = ScalePaths64(paths, scale);
      ClipperOffset co = new ClipperOffset(miterLimit, scale * arcTolerance);
      co.AddPaths(tmp, joinType, endType);
      co.Execute(delta * scale, tmp); // reuse 'tmp' to receive (scaled) solution
      return ScalePathsD(tmp, 1 / scale);
    }

    public static PathsPoint RectClip(Rectangle rect, PathsPoint paths)
    {
      if (rect.width <= 0 || rect.height <= 0 || paths.Count == 0) return new PathsPoint();
      RectClip64 rc = new RectClip64(rect);
      return rc.Execute(paths);
    }

    public static PathsPoint RectClip(Rectangle rect, PathPoint path)
    {
      if (rect.width <= 0 || rect.height <= 0 || path.Count == 0) return new PathsPoint();
      PathsPoint tmp = new PathsPoint { path };
      return RectClip(rect, tmp);
    }
    
    public static PathsVector2 RectClip(Box rect, PathsVector2 paths, int precision = 2)
    {
      InternalClipper.CheckPrecision(precision);
      if (rect.width <= 0 || rect.height <= 0 || paths.Count == 0) return new PathsVector2();
      float scale = float.Pow(10, precision);
      Rectangle r = ScaleRect(rect, scale);
      PathsPoint tmpPath = ScalePaths64(paths, scale);
      RectClip64 rc = new RectClip64(r);
      tmpPath = rc.Execute(tmpPath);
      return ScalePathsD(tmpPath, 1 / scale);
    }

    public static PathsVector2 RectClip(Box rect, PathVector2 path, int precision = 2)
    {
      if (rect.width <= 0 || rect.height <= 0 || path.Count == 0) return new PathsVector2();
      PathsVector2 tmp = new PathsVector2 { path };
      return RectClip(rect, tmp, precision);
    }
    public static PathsPoint RectClipLines(Rectangle rect, PathsPoint paths)
    {
      if (rect.width <= 0 || rect.height <= 0 || paths.Count == 0) return new PathsPoint();
      RectClipLines64 rc = new RectClipLines64(rect);
      return rc.Execute(paths);
    }

    public static PathsPoint RectClipLines(Rectangle rect, PathPoint path)
    {
      if (rect.width <= 0 || rect.height <= 0 || path.Count == 0) return new PathsPoint();
      PathsPoint tmp = new PathsPoint { path };
      return RectClipLines(rect, tmp);
    }

    public static PathsVector2 RectClipLines(Box rect, 
      PathsVector2 paths, int precision = 2)
    {
      InternalClipper.CheckPrecision(precision);
      if (rect.width <= 0 || rect.height <= 0 || paths.Count == 0) return new PathsVector2();
      float scale = float.Pow(10, precision);
      Rectangle r = ScaleRect(rect, scale);
      PathsPoint tmpPath = ScalePaths64(paths, scale);
      RectClipLines64 rc = new RectClipLines64(r);
      tmpPath = rc.Execute(tmpPath);
      return ScalePathsD(tmpPath, 1 / scale);
    }
    public static PathsVector2 RectClipLines(Box rect, PathVector2 path, int precision = 2)
    {
      if (rect.width <= 0 || rect.height <= 0 || path.Count == 0) return new PathsVector2();
      PathsVector2 tmp = new PathsVector2 { path };
      return RectClipLines(rect, tmp, precision);
    }
    public static PathsPoint MinkowskiSum(PathPoint pattern, PathPoint path, bool isClosed)
    {
      return Minkowski.Sum(pattern, path, isClosed);
    }

    public static PathsVector2 MinkowskiSum(PathVector2 pattern, PathVector2 path, bool isClosed)
    {
      return Minkowski.Sum(pattern, path, isClosed);
    }

    public static PathsPoint MinkowskiDiff(PathPoint pattern, PathPoint path, bool isClosed)
    {
      return Minkowski.Diff(pattern, path, isClosed);
    }

    public static PathsVector2 MinkowskiDiff(PathVector2 pattern, PathVector2 path, bool isClosed)
    {
      return Minkowski.Diff(pattern, path, isClosed);
    }

    public static float Area(PathPoint path)
    {
      // https://en.wikipedia.org/wiki/Shoelace_formula
      float a = 0.0f;
      int cnt = path.Count;
      if (cnt < 3) return 0.0f;
      Point prevPt = path[cnt - 1];
      foreach (Point pt in path)
      {
        a += (float) (prevPt.Y + pt.Y) * (prevPt.X - pt.X);
        prevPt = pt;
      }
      return a * 0.5f;
    }

    public static float Area(PathsPoint paths)
    {
      float a = 0.0f;
      foreach (PathPoint path in paths)
        a += Area(path);
      return a;
    }

    public static float Area(PathVector2 path)
    {
      float a = 0.0f;
      int cnt = path.Count;
      if (cnt < 3) return 0.0f;
      Vector2 prevPt = path[cnt - 1];
      foreach (Vector2 pt in path)
      {
        a += (prevPt.Y + pt.Y) * (prevPt.X - pt.X);
        prevPt = pt;
      }
      return a * 0.5f;
    }

    public static float Area(PathsVector2 paths)
    {
      float a = 0.0f;
      foreach (PathVector2 path in paths)
        a += Area(path);
      return a;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsPositive(PathPoint poly)
    {
      return Area(poly) >= 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsPositive(PathVector2 poly)
    {
      return Area(poly) >= 0;
    }

    public static string Path64ToString(PathPoint path)
    {
      string result = "";
      foreach (Point pt in path)
        result += pt.ToString();
      return result + '\n';
    }
    public static string Paths64ToString(PathsPoint paths)
    {
      string result = "";
      foreach (PathPoint path in paths)
        result += Path64ToString(path);
      return result;
    }
    public static string PathDToString(PathVector2 path)
    {
      string result = "";
      foreach (Vector2 pt in path)
        result += pt.ToString();
      return result + '\n';
    }
    public static string PathsDToString(PathsVector2 paths)
    {
      string result = "";
      foreach (PathVector2 path in paths)
        result += PathDToString(path);
      return result;
    }
    public static PathPoint OffsetPath(PathPoint path, int dx, int dy)
    {
      PathPoint result = new PathPoint(path.Count);
      foreach (Point pt in path)
        result.Add(new Point(pt.X + dx, pt.Y + dy));
      return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Point ScalePoint(Point pt, float scale)
    {
      return new((int) Math.Round(pt.X * scale, MidpointRounding.AwayFromZero), (int) Math.Round(pt.Y * scale, MidpointRounding.AwayFromZero));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2 ScaleVector2(Point pt, float scale)
    {
      return new(pt.X * scale, pt.Y * scale);
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Rectangle ScaleRect(Box rec, float scale)
    {
      Rectangle result = new Rectangle()
      {
        X = (int) (rec.Left * scale),
        Y = (int) (rec.Top * scale),
        width = (int) ((rec.Right -  rec.Left) * scale),
        height = (int) ((rec.Bottom - rec.Top) * scale)
      };
      return result;
    }

    public static PathPoint ScalePath(PathPoint path, float scale)
    {
      if (InternalClipper.IsAlmostZero(scale - 1)) return path;
      PathPoint result = new PathPoint(path.Count);
      foreach (Point pt in path)
        result.Add((pt.ToVector2() * scale).ToPoint());
      return result;
    }

    public static PathsPoint ScalePaths(PathsPoint paths, float scale)
    {
      if (InternalClipper.IsAlmostZero(scale - 1)) return paths;
      PathsPoint result = new PathsPoint(paths.Count);
      foreach (PathPoint path in paths)
        result.Add(ScalePath(path, scale));
      return result;
    }

    public static PathVector2 ScalePath(PathVector2 path, float scale)
    {
      if (InternalClipper.IsAlmostZero(scale - 1)) return path;
      PathVector2 result = new PathVector2(path.Count);
      foreach (Vector2 pt in path)
        result.Add(pt * scale);
      return result;
    }

    public static PathsVector2 ScalePaths(PathsVector2 paths, float scale)
    {
      if (InternalClipper.IsAlmostZero(scale - 1)) return paths;
      PathsVector2 result = new PathsVector2(paths.Count);
      foreach (PathVector2 path in paths)
        result.Add(ScalePath(path, scale));
      return result;
    }

    // Unlike ScalePath, both ScalePath64 & ScalePathD also involve type conversion
    public static PathPoint ScalePath64(PathVector2 path, float scale)
    {
      int cnt = path.Count;
      PathPoint res = new PathPoint(cnt);
      foreach (Vector2 pt in path)
        res.Add((pt * scale).ToPoint());
      return res;
    }

    public static PathsPoint ScalePaths64(PathsVector2 paths, float scale)
    {
      int cnt = paths.Count;
      PathsPoint res = new PathsPoint(cnt);
      foreach (PathVector2 path in paths)
        res.Add(ScalePath64(path, scale));
      return res;
    }

    public static PathVector2 ScalePathD(PathPoint path, float scale)
    {
      int cnt = path.Count;
      PathVector2 res = new PathVector2(cnt);
      foreach (Point pt in path)
        res.Add(pt.ToVector2() * scale);
      return res;
    }

    public static PathsVector2 ScalePathsD(PathsPoint paths, float scale)
    {
      int cnt = paths.Count;
      PathsVector2 res = new PathsVector2(cnt);
      foreach (PathPoint path in paths)
        res.Add(ScalePathD(path, scale));
      return res;
    }

    // The static functions Path64 and PathD convert path types without scaling
    public static PathPoint Path64(PathVector2 path)
    {
      PathPoint result = new PathPoint(path.Count);
      foreach (Vector2 pt in path)
        result.Add(pt.ToPoint());
      return result;
    }

    public static PathsPoint Paths64(PathsVector2 paths)
    {
      PathsPoint result = new PathsPoint(paths.Count);
      foreach (PathVector2 path in paths)
        result.Add(Path64(path));
      return result;
    }

    public static PathsVector2 PathsD(PathsPoint paths)
    {
      PathsVector2 result = new PathsVector2(paths.Count);
      foreach (PathPoint path in paths)
        result.Add(PathD(path));
      return result;
    }

    public static PathVector2 PathD(PathPoint path)
    {
      PathVector2 result = new PathVector2(path.Count);
      foreach (Point pt in path)
        result.Add(pt.ToVector2());
      return result;
    }

    public static PathPoint TranslatePath(PathPoint path, int dx, int dy)
    {
      PathPoint result = new PathPoint(path.Count);
      foreach (Point pt in path)
        result.Add(new Point(pt.X + dx, pt.Y + dy));
      return result;
    }

    public static PathsPoint TranslatePaths(PathsPoint paths, int dx, int dy)
    {
      PathsPoint result = new PathsPoint(paths.Count);
      foreach (PathPoint path in paths)
        result.Add(OffsetPath(path, dx, dy));
      return result;
    }

    public static PathVector2 TranslatePath(PathVector2 path, float dx, float dy)
    {
      PathVector2 result = new PathVector2(path.Count);
      foreach (Vector2 pt in path)
        result.Add(new Vector2(pt.X + dx, pt.Y + dy));
      return result;
    }

    public static PathsVector2 TranslatePaths(PathsVector2 paths, float dx, float dy)
    {
      PathsVector2 result = new PathsVector2(paths.Count);
      foreach (PathVector2 path in paths)
        result.Add(TranslatePath(path, dx, dy));
      return result;
    }

    public static PathPoint ReversePath(PathPoint path)
    {
      PathPoint result = new PathPoint(path);
      result.Reverse();
      return result;
    }

    public static PathVector2 ReversePath(PathVector2 path)
    {
      PathVector2 result = new PathVector2(path);
      result.Reverse();
      return result;
    }

    public static PathsPoint ReversePaths(PathsPoint paths)
    {
      PathsPoint result = new PathsPoint(paths.Count);
      foreach (PathPoint t in paths)
        result.Add(ReversePath(t));

      return result;
    }

    public static PathsVector2 ReversePaths(PathsVector2 paths)
    {
      PathsVector2 result = new PathsVector2(paths.Count);
      foreach (PathVector2 path in paths)
        result.Add(ReversePath(path));
      return result;
    }

    public static Rectangle GetBounds(PathPoint path)
    {
      Rectangle result = InvalidRectangle;
      foreach (Point pt in path)
      {
        if (pt.X < result.Left) result.X = pt.X;
        if (pt.X > result.Right) result.width = pt.X - result.X;
        if (pt.Y < result.Top) result.Y = pt.Y;
        if (pt.Y > result.Bottom) result.height = pt.Y - result.Y;
      }
      return result.X == int.MaxValue ? new Rectangle() : result;
    }

    public static Rectangle GetBounds(PathsPoint paths)
    {
      Rectangle result = InvalidRectangle;
      foreach (PathPoint path in paths)
        foreach (Point pt in path)
        {
          if (pt.X < result.Left) result.X = pt.X;
          if (pt.X > result.Right) result.width = pt.X - result.X;
          if (pt.Y < result.Top) result.Y = pt.Y;
          if (pt.Y > result.Bottom) result.height = pt.Y - result.Y;
        }
      return result.X == int.MaxValue ? new Rectangle() : result;
    }

    public static Box GetBounds(PathVector2 path)
    {
      Box result = InvalidBox;
      foreach (Vector2 pt in path)
      {
        if (pt.X < result.Left) result.X = pt.X;
        if (pt.X > result.Right) result.width = pt.X - result.X;
        if (pt.Y < result.Top) result.Y = pt.Y;
        if (pt.Y > result.Bottom) result.height = pt.Y - result.Y;
      }
      return Math.Abs(result.Left - float.MaxValue) < InternalClipper.floatingPointTolerance ? new Box() : result;
    }

    public static Box GetBounds(PathsVector2 paths)
    {
      Box result = InvalidBox;
      foreach (PathVector2 path in paths)
        foreach (Vector2 pt in path)
        {
          if (pt.X < result.Left) result.X = pt.X;
          if (pt.X > result.Right) result.width = pt.X - result.X;
          if (pt.Y < result.Top) result.Y = pt.Y;
          if (pt.Y > result.Bottom) result.height = pt.Y - result.Y;
        }
      return Math.Abs(result.Left - float.MaxValue) < InternalClipper.floatingPointTolerance ? new Box() : result;
    }

    public static PathPoint MakePath(int[] arr)
    {
      int len = arr.Length / 2;
      PathPoint p = new PathPoint(len);
      for (int i = 0; i < len; i++)
        p.Add(new Point(arr[i * 2], arr[i * 2 + 1]));
      return p;
    }

    public static PathVector2 MakePath(float[] arr)
    {
      int len = arr.Length / 2;
      PathVector2 p = new PathVector2(len);
      for (int i = 0; i < len; i++)
        p.Add(new Vector2(arr[i * 2], arr[i * 2 + 1]));
      return p;
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Sqr(float val)
    {
      return val * val;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Sqr(int val)
    {
      return (float) val * (float) val;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float DistanceSqr(Point pt1, Point pt2)
    {
      return Sqr(pt1.X - pt2.X) + Sqr(pt1.Y - pt2.Y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Point MidPoint(Point pt1, Point pt2)
    {
      return new Point((pt1.X + pt2.X) / 2, (pt1.Y + pt2.Y) / 2);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vector2 MidPoint(Vector2 pt1, Vector2 pt2)
    {
      return new Vector2((pt1.X + pt2.X) / 2, (pt1.Y + pt2.Y) / 2);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void InflateRect(ref Rectangle rec, int dx, int dy)
    {
      rec.X -= dx;
      rec.width += dx * 2;
      rec.Y -= dy;
      rec.height += dy * 2;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void InflateRect(ref Box rec, float dx, float dy)
    {
      rec.X -= dx;
      rec.width += dx * 2;
      rec.Y -= dy;
      rec.height += dy * 2;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool PointsNearEqual(Vector2 pt1, Vector2 pt2, float distanceSqrd)
    {
      return Sqr(pt1.X - pt2.X) + Sqr(pt1.Y - pt2.Y) < distanceSqrd;
    }

    public static PathVector2 StripNearDuplicates(PathVector2 path,
        float minEdgeLenSqrd, bool isClosedPath)
    {
      int cnt = path.Count;
      PathVector2 result = new PathVector2(cnt);
      if (cnt == 0) return result;
      Vector2 lastPt = path[0];
      result.Add(lastPt);
      for (int i = 1; i < cnt; i++)
        if (!PointsNearEqual(lastPt, path[i], minEdgeLenSqrd))
        {
          lastPt = path[i];
          result.Add(lastPt);
        }

      if (isClosedPath && PointsNearEqual(lastPt, result[0], minEdgeLenSqrd))
      {
        result.RemoveAt(result.Count - 1);
      }

      return result;
    }

    public static PathPoint StripDuplicates(PathPoint path, bool isClosedPath)
    {
      int cnt = path.Count;
      PathPoint result = new PathPoint(cnt);
      if (cnt == 0) return result;
      Point lastPt = path[0];
      result.Add(lastPt);
      for (int i = 1; i < cnt; i++)
        if (lastPt != path[i])
        {
          lastPt = path[i];
          result.Add(lastPt);
        }
      if (isClosedPath && lastPt == result[0])
        result.RemoveAt(result.Count - 1);
      return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddPolyNodeToPaths(PolyPath64 polyPath, PathsPoint paths)
    {
      if (polyPath.Polygon!.Count > 0)
        paths.Add(polyPath.Polygon);
      for (int i = 0; i < polyPath.Count; i++)
        AddPolyNodeToPaths((PolyPath64) polyPath._childs[i], paths);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static PathsPoint PolyTreeToPaths64(PolyTree64 polyTree)
    {
      PathsPoint result = new PathsPoint();
      for (int i = 0; i < polyTree.Count; i++)
        AddPolyNodeToPaths((PolyPath64) polyTree._childs[i], result);
      return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void AddPolyNodeToPathsD(PolyPathD polyPath, PathsVector2 paths)
    {
      if (polyPath.Polygon!.Count > 0)
        paths.Add(polyPath.Polygon);
      for (int i = 0; i < polyPath.Count; i++)
        AddPolyNodeToPathsD((PolyPathD) polyPath._childs[i], paths);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static PathsVector2 PolyTreeToPathsD(PolyTreeD polyTree)
    {
      PathsVector2 result = new PathsVector2();
      foreach (PolyPathD polyPathBase in polyTree)
      {
        PolyPathD p = (PolyPathD)polyPathBase;
        AddPolyNodeToPathsD(p, result);
      }

      return result;
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float PerpendicDistFromLineSqrd(Vector2 pt, Vector2 line1, Vector2 line2)
    {
      float a = pt.X - line1.X;
      float b = pt.Y - line1.Y;
      float c = line2.X - line1.X;
      float d = line2.Y - line1.Y;
      if (c == 0 && d == 0) return 0;
      return Sqr(a * d - c * b) / (c * c + d * d);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float PerpendicDistFromLineSqrd(Point pt, Point line1, Point line2)
    {
      float a = (float) pt.X - line1.X;
      float b = (float) pt.Y - line1.Y;
      float c = (float) line2.X - line1.X;
      float d = (float) line2.Y - line1.Y;
      if (c == 0 && d == 0) return 0;
      return Sqr(a * d - c * b) / (c * c + d * d);
    }

    internal static void RDP(PathPoint path, int begin, int end, float epsSqrd, List<bool> flags)
    {
      while (true)
      {
        int idx = 0;
        float max_d = 0;
        while (end > begin && path[begin] == path[end]) flags[end--] = false;
        for (int i = begin + 1; i < end; ++i)
        {
          // PerpendicDistFromLineSqrd - avoids expensive Sqrt()
          float d = PerpendicDistFromLineSqrd(path[i], path[begin], path[end]);
          if (d <= max_d) continue;
          max_d = d;
          idx = i;
        }

        if (max_d <= epsSqrd) return;
        flags[idx] = true;
        if (idx > begin + 1) RDP(path, begin, idx, epsSqrd, flags);
        if (idx < end - 1)
        {
          begin = idx;
          continue;
        }

        break;
      }
    }

    public static PathPoint RamerDouglasPeucker(PathPoint path, float epsilon)
    {
      int len = path.Count;
      if (len < 5) return path;
      List<bool> flags = new List<bool>(new bool[len]) { [0] = true, [len - 1] = true };
      RDP(path, 0, len - 1, Sqr(epsilon), flags);
      PathPoint result = new PathPoint(len);
      for (int i = 0; i < len; ++i)
        if (flags[i]) result.Add(path[i]);
      return result;
    }

    public static PathsPoint RamerDouglasPeucker(PathsPoint paths, float epsilon)
    {
      PathsPoint result = new PathsPoint(paths.Count);
      foreach (PathPoint path in paths)
        result.Add(RamerDouglasPeucker(path, epsilon));
      return result;
    }

    internal static void RDP(PathVector2 path, int begin, int end, float epsSqrd, List<bool> flags)
    {
      while (true)
      {
        int idx = 0;
        float max_d = 0;
        while (end > begin && path[begin] == path[end]) flags[end--] = false;
        for (int i = begin + 1; i < end; ++i)
        {
          // PerpendicDistFromLineSqrd - avoids expensive Sqrt()
          float d = PerpendicDistFromLineSqrd(path[i], path[begin], path[end]);
          if (d <= max_d) continue;
          max_d = d;
          idx = i;
        }

        if (max_d <= epsSqrd) return;
        flags[idx] = true;
        if (idx > begin + 1) RDP(path, begin, idx, epsSqrd, flags);
        if (idx < end - 1)
        {
          begin = idx;
          continue;
        }

        break;
      }
    }

    public static PathVector2 RamerDouglasPeucker(PathVector2 path, float epsilon)
    {
      int len = path.Count;
      if (len < 5) return path;
      List<bool> flags = new List<bool>(new bool[len]) { [0] = true, [len - 1] = true };
      RDP(path, 0, len - 1, Sqr(epsilon), flags);
      PathVector2 result = new PathVector2(len);
      for (int i = 0; i < len; ++i)
        if (flags[i]) result.Add(path[i]);
      return result;
    }

    public static PathsVector2 RamerDouglasPeucker(PathsVector2 paths, float epsilon)
    {
      PathsVector2 result = new PathsVector2(paths.Count);
      foreach (PathVector2 path in paths)
        result.Add(RamerDouglasPeucker(path, epsilon));
      return result;
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int GetNext(int current, int high, ref bool[] flags)
    {
      ++current;
      while (current <= high && flags[current]) ++current;
      if (current <= high) return current;
      current = 0;
      while (flags[current]) ++current;
      return current;
    }

    private static int GetPrior(int current, int high, ref bool[] flags)
    {
      if (current == 0) current = high;
      else --current;
      while (current > 0 && flags[current]) --current;
      if (!flags[current]) return current;
      current = high;
      while (flags[current]) --current;
      return current;
    }

      public static PathPoint SimplifyPath(PathPoint path,
      float epsilon, bool isClosedPath = true)
    {
      int len = path.Count, high = len - 1;
      float epsSqr = Sqr(epsilon);
      if (len < 4) return path;

      bool[] flags = new bool[len];
      float[] dsq = new float[len];
      int curr = 0;

      if (isClosedPath)
      {
        dsq[0] = PerpendicDistFromLineSqrd(path[0], path[high], path[1]);
        dsq[high] = PerpendicDistFromLineSqrd(path[high], path[0], path[high - 1]);
      }
      else
      {
        dsq[0] = float.MaxValue;
        dsq[high] = float.MaxValue;
      }

      for (int i = 1; i < high; ++i)
        dsq[i] = PerpendicDistFromLineSqrd(path[i], path[i - 1], path[i + 1]);

      for (; ; )
      {
        if (dsq[curr] > epsSqr)
        {
          int start = curr;
          do
          {
            curr = GetNext(curr, high, ref flags);
          } while (curr != start && dsq[curr] > epsSqr);
          if (curr == start) break;
        }

        int prev = GetPrior(curr, high, ref flags);
        int next = GetNext(curr, high, ref flags);
        if (next == prev) break;

        int prior2;
        if (dsq[next] < dsq[curr])
        {
          prior2 = prev;
          prev = curr;
          curr = next;
          next = GetNext(next, high, ref flags);
        }
        else
          prior2 = GetPrior(prev, high, ref flags);

        flags[curr] = true;
        curr = next;
        next = GetNext(next, high, ref flags);
        if (isClosedPath || ((curr != high) && (curr != 0)))
          dsq[curr] = PerpendicDistFromLineSqrd(path[curr], path[prev], path[next]);
        if (isClosedPath || ((prev != 0) && (prev != high)))
          dsq[prev] = PerpendicDistFromLineSqrd(path[prev], path[prior2], path[curr]);
      }
      PathPoint result = new PathPoint(len);
      for (int i = 0; i < len; i++)
        if (!flags[i]) result.Add(path[i]);
      return result;
    }

    public static PathsPoint SimplifyPaths(PathsPoint paths,
      float epsilon, bool isClosedPaths = true)
    {
      PathsPoint result = new PathsPoint(paths.Count);
      foreach (PathPoint path in paths)
        result.Add(SimplifyPath(path, epsilon, isClosedPaths));
      return result;
    }

    public static PathVector2 SimplifyPath(PathVector2 path,
      float epsilon, bool isClosedPath = true)
    {
      int len = path.Count, high = len - 1;
      float epsSqr = Sqr(epsilon);
      if (len < 4) return path;

      bool[] flags = new bool[len];
      float[] dsq = new float[len];
      int curr = 0;
      if (isClosedPath)
      {
        dsq[0] = PerpendicDistFromLineSqrd(path[0], path[high], path[1]);
        dsq[high] = PerpendicDistFromLineSqrd(path[high], path[0], path[high - 1]);
      }
      else
      {
        dsq[0] = float.MaxValue;
        dsq[high] = float.MaxValue;
      }
      for (int i = 1; i < high; ++i)
        dsq[i] = PerpendicDistFromLineSqrd(path[i], path[i - 1], path[i + 1]);

      for (; ; )
      {
        if (dsq[curr] > epsSqr)
        {
          int start = curr;
          do
          {
            curr = GetNext(curr, high, ref flags);
          } while (curr != start && dsq[curr] > epsSqr);
          if (curr == start) break;
        }

        int prev = GetPrior(curr, high, ref flags);
        int next = GetNext(curr, high, ref flags);
        if (next == prev) break;

        int prior2;
        if (dsq[next] < dsq[curr])
        {
          prior2 = prev;
          prev = curr;
          curr = next;
          next = GetNext(next, high, ref flags);
        }
        else 
          prior2 = GetPrior(prev, high, ref flags);

        flags[curr] = true;
        curr = next;
        next = GetNext(next, high, ref flags);
        if (isClosedPath || ((curr != high) && (curr != 0)))
          dsq[curr] = PerpendicDistFromLineSqrd(path[curr], path[prev], path[next]);
        if (isClosedPath || ((prev != 0) && (prev != high)))
          dsq[prev] = PerpendicDistFromLineSqrd(path[prev], path[prior2], path[curr]);
      }
      PathVector2 result = new PathVector2(len);
      for (int i = 0; i < len; i++)
        if (!flags[i]) result.Add(path[i]);
      return result;
    }

    public static PathsVector2 SimplifyPaths(PathsVector2 paths,
      float epsilon, bool isClosedPath = true)
    {
      PathsVector2 result = new PathsVector2(paths.Count);
      foreach (PathVector2 path in paths)
        result.Add(SimplifyPath(path, epsilon, isClosedPath));
      return result;
    }

    public static PathPoint TrimCollinear(PathPoint path, bool isOpen = false)
    {
      int len = path.Count;
      int i = 0;
      if (!isOpen)
      {
        while (i < len - 1 && 
          InternalClipper.IsCollinear(path[len - 1], path[i], path[i + 1])) i++;
        while (i < len - 1 && InternalClipper.IsCollinear(path[len - 2], path[len - 1], path[i])) len--;
      }

      if (len - i < 3)
      {
        if (!isOpen || len < 2 || path[0] == path[1])
          return new PathPoint();
        return path;
      }

      PathPoint result = new PathPoint(len - i);
      Point last = path[i];
      result.Add(last);
      for (i++; i < len - 1; i++)
      {
        if (InternalClipper.IsCollinear(last, path[i], path[i + 1])) continue;
        last = path[i];
        result.Add(last);
      }

      if (isOpen)
        result.Add(path[len - 1]);
      else if (!InternalClipper.IsCollinear(last, path[len - 1], result[0]))
        result.Add(path[len - 1]);
      else
      {
        while (result.Count > 2 && InternalClipper.IsCollinear(
                 result[result.Count - 1], result[result.Count - 2], result[0]))
        {
          result.RemoveAt(result.Count - 1);
        }
        if (result.Count < 3)
          result.Clear();
      }
      return result;
    }

    public static PathVector2 TrimCollinear(PathVector2 path, int precision, bool isOpen = false)
    {
      InternalClipper.CheckPrecision(precision);
      float scale = float.Pow(10, precision);
      PathPoint p = ScalePath64(path, scale);
      p = TrimCollinear(p, isOpen);
      return ScalePathD(p, 1 / scale);
    }

    public static PointInPolygonResult PointInPolygon(Point pt, PathPoint polygon)
    {
      return InternalClipper.PointInPolygon(pt, polygon);
    }

    public static PointInPolygonResult PointInPolygon(Vector2 pt, 
      PathVector2 polygon, int precision = 2)
    {
      InternalClipper.CheckPrecision(precision);
      float scale = float.Pow(10, precision);
      Point p = (pt * scale).ToPoint();
      PathPoint path = ScalePath64(polygon, scale);
      return InternalClipper.PointInPolygon(p, path);
    }

    public static PathPoint Ellipse(Point center,
      float radiusX, float radiusY = 0, int steps = 0)
    {
      if (radiusX <= 0) return new PathPoint();
      if (radiusY <= 0) radiusY = radiusX;
      if (steps <= 2)
        steps = (int) Math.Ceiling(MathF.PI * Math.Sqrt((radiusX + radiusY) / 2));

      float si = float.Sin(2 * MathF.PI / steps);
      float co = float.Cos(2 * MathF.PI / steps);
      float dx = co, dy = si;
      PathPoint result = new PathPoint(steps) { new Point(center.X + (int)radiusX, center.Y) };
      for (int i = 1; i < steps; ++i)
      {
        result.Add(new Point(center.X + (int)(radiusX * dx), center.Y + (int)(radiusY * dy)));
        float x = dx * co - dy * si;
        dy = dy * co + dx * si;
        dx = x;
      }
      return result;
    }

    public static PathVector2 Ellipse(Vector2 center,
      float radiusX, float radiusY = 0, int steps = 0)
    {
      if (radiusX <= 0) return new PathVector2();
      if (radiusY <= 0) radiusY = radiusX;
      if (steps <= 2)
        steps = (int) Math.Ceiling(MathF.PI * Math.Sqrt((radiusX + radiusY) / 2));

      float si = float.Sin(2 * MathF.PI / steps);
      float co = float.Cos(2 * MathF.PI / steps);
      float dx = co, dy = si;
      PathVector2 result = new PathVector2(steps) { new Vector2(center.X + radiusX, center.Y) };
      for (int i = 1; i < steps; ++i)
      {
        result.Add(new Vector2(center.X + radiusX * dx, center.Y + radiusY * dy));
        float x = dx * co - dy * si;
        dy = dy * co + dx * si;
        dx = x;
      }
      return result;
    }

    private static void ShowPolyPathStructure(PolyPath64 pp, int level)
    {
      string spaces = new string(' ', level * 2);
      string caption = (pp.IsHole ? "Hole " : "Outer ");
      if (pp.Count == 0)
      {
        Console.WriteLine(spaces + caption);
      }
      else
      {
        Console.WriteLine(spaces + caption + $"({pp.Count})");
        foreach (PolyPath64 child in pp) { ShowPolyPathStructure(child, level + 1); }
      }
    }

    public static void ShowPolyTreeStructure(PolyTree64 polytree)
    {
      Console.WriteLine("Polytree Root");
      foreach (PolyPath64 child in polytree) { ShowPolyPathStructure(child, 1); }
    }

    private static void ShowPolyPathStructure(PolyPathD pp, int level)
    {
      string spaces = new string(' ', level * 2);
      string caption = (pp.IsHole ? "Hole " : "Outer ");
      if (pp.Count == 0)
      {
        Console.WriteLine(spaces + caption);
      }
      else
      {
        Console.WriteLine(spaces + caption + $"({pp.Count})");
        foreach (PolyPathD child in pp) { ShowPolyPathStructure(child, level + 1); }
      }
    }

    public static void ShowPolyTreeStructure(PolyTreeD polytree)
    {
      Console.WriteLine("Polytree Root");
      foreach (PolyPathD child in polytree) { ShowPolyPathStructure(child, 1); }
    }

    public static TriangulateResult Triangulate(PathsPoint pp, out PathsPoint solution, bool useDelaunay = true)
    {
      Delaunay d = new Delaunay(useDelaunay);
      return d.Execute(pp, out solution);
    }

    public static TriangulateResult Triangulate(PathsVector2 pp, int decPlaces, out PathsVector2 solution, bool useDelaunay = true)
    {
      float scale;
      if (decPlaces <= 0) scale = 1.0f;
      else if (decPlaces > 8) scale = float.Pow(10.0f, 8.0f);
      else scale = float.Pow(10.0f, decPlaces);

      PathsPoint pp64 = Clipper.ScalePaths64(pp, scale);

      Delaunay d = new Delaunay(useDelaunay);
      TriangulateResult result = d.Execute(pp64, out PathsPoint sol64);
      if (result == TriangulateResult.success)
        solution = Clipper.ScalePathsD(sol64, 1.0f / scale);
      else
        solution = new PathsVector2();
      return result;
    }

    public static PathPoint AsPath(Rectangle rect)
    {
      PathPoint result =
      [
        new Point(rect.X, rect.Y),
        new Point(rect.X + rect.width, rect.Y),
        new Point(rect.X + rect.width, rect.Y + rect.height),
        new Point(rect.X, rect.Y + rect.height)
      ];
      return result;
    }

    public static PathVector2 AsPath(Box box)
    {
      PathVector2 result =
      [
        new Vector2(box.Left, box.Top),
        new Vector2(box.Right, box.Top),
        new Vector2(box.Right, box.Bottom),
        new Vector2(box.Left, box.Bottom)
      ];
      return result;
    }
  } // Clipper
} // namespace