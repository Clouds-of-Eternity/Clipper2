/*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Date      :  10 October 2024                                                 *
* Website   :  https://www.angusj.com                                          *
* Copyright :  Angus Johnson 2010-2024                                         *
* Purpose   :  Minkowski Sum and Difference                                    *
* License   :  https://www.boost.org/LICENSE_1_0.txt                           *
*******************************************************************************/

#nullable enable
using System;
using EternityWorks;

namespace Clipper2Lib
{
  public static class Minkowski
  {
    private static PathsPoint MinkowskiInternal(PathPoint pattern, PathPoint path, bool isSum, bool isClosed)
    {
      int delta = isClosed ? 0 : 1;
      int patLen = pattern.Count, pathLen = path.Count;
      PathsPoint tmp = new PathsPoint(pathLen);

      foreach (Point pathPt in path)
      {
        PathPoint path2 = new PathPoint(patLen);
        if (isSum)
        {
          foreach (Point basePt in pattern)
            path2.Add(pathPt + basePt);
        }
        else
        {
          foreach (Point basePt in pattern)
            path2.Add(pathPt - basePt);
        }
        tmp.Add(path2);
      }

      PathsPoint result = new PathsPoint((pathLen - delta) * patLen);
      int g = isClosed ? pathLen - 1 : 0;

      int h = patLen - 1;
      for (int i = delta; i < pathLen; i++)
      {
        for (int j = 0; j < patLen; j++)
        {
          PathPoint quad = new PathPoint(4)
          {
            tmp[g][h], tmp[i][h], tmp[i][j], tmp[g][j]
          };
          if (!Clipper.IsPositive(quad))
            result.Add(Clipper.ReversePath(quad));
          else
            result.Add(quad);
          h = j;
        }
        g = i;
      }
      return result;
    }

    public static PathsPoint Sum(PathPoint pattern, PathPoint path, bool isClosed)
    {
      return Clipper.Union(MinkowskiInternal(pattern, path, true, isClosed), FillRule.NonZero);
    }

    public static PathsVector2 Sum(PathVector2 pattern, PathVector2 path, bool isClosed, int decimalPlaces = 2)
    {
      float scale = float.Pow(10, decimalPlaces);
      PathsPoint tmp = Clipper.Union(MinkowskiInternal(Clipper.ScalePath64(pattern, scale),
        Clipper.ScalePath64(path, scale), true, isClosed), FillRule.NonZero);
      return Clipper.ScalePathsD(tmp, 1 / scale);
    }

    public static PathsPoint Diff(PathPoint pattern, PathPoint path, bool isClosed)
    {
      return Clipper.Union(MinkowskiInternal(pattern, path, false, isClosed), FillRule.NonZero);
    }

    public static PathsVector2 Diff(PathVector2 pattern, PathVector2 path, bool isClosed, int decimalPlaces = 2)
    {
      float scale = float.Pow(10, decimalPlaces);
      PathsPoint tmp = Clipper.Union(MinkowskiInternal(Clipper.ScalePath64(pattern, scale),
        Clipper.ScalePath64(path, scale), false, isClosed), FillRule.NonZero);
      return Clipper.ScalePathsD(tmp, 1 / scale);
    }

  }

} // namespace