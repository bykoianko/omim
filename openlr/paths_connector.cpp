#include "openlr/paths_connector.hpp"
#include "openlr/helpers.hpp"

#include "indexer/feature_data.hpp"
#include "indexer/ftypes_matcher.hpp"

#include "base/checked_cast.hpp"
#include "base/stl_iterator.hpp"

#include <algorithm>
#include <map>
#include <queue>
#include <tuple>

using namespace std;

namespace openlr
{
namespace
{
size_t IntersectionLen(Graph::EdgeVector a, Graph::EdgeVector b)
{
  sort(begin(a), end(a));
  sort(begin(b), end(b));
  return set_intersection(begin(a), end(a), begin(b), end(b), CounterIterator()).GetCount();
}

bool PrefEqualsSuff(Graph::EdgeVector const & a, Graph::EdgeVector const & b, size_t const len)
{
  ASSERT_LESS_OR_EQUAL(len, a.size(), ());
  ASSERT_LESS_OR_EQUAL(len, b.size(), ());
  return equal(end(a) - len, end(a), begin(b));
}

// Returns a length of the longest suffix of |a| that matches any prefix of |b|.
// Neither |a| nor |b| can contain several repetitions of any edge.
// Returns -1 if |a| intersection |b| is not equal to some suffix of |a| and some prefix of |b|.
int32_t PathOverlappingLen(Graph::EdgeVector const & a, Graph::EdgeVector const & b)
{
  auto const len = IntersectionLen(a, b);
  if (PrefEqualsSuff(a, b, len))
    return base::checked_cast<int32_t>(len);
  return -1;
}

// @TODO Rename this method.
Score2 ValidatePath(Graph::EdgeVector const & path,
                  double const distanceToNextPoint,
                  double const pathLengthTolerance)
{

  double pathLen = 0.0;
  for (auto const & e : path)
    pathLen += EdgeLength(e);

  // @TODO Rename this value.
//  double pathDiffPercent = AbsDifference(static_cast<double>(distanceToNextPoint), pathLen) /
//                           static_cast<double>(distanceToNextPoint);
//
//  LOG(LDEBUG, ("Validating path:", LogAs2GisPath(path)));

  double const pathDiffRatio =
      AbsDifference(distanceToNextPoint, pathLen) / max(distanceToNextPoint, pathLen);

  return 100.0 * (1.0 - pathDiffRatio);

//  if (pathDiffPercent > pathLengthTolerance)
//  {
//    LOG(LDEBUG,
//        ("Shortest path doest not meet required length constraints, error:", pathDiffPercent));
//    return false;
//  }
//
//  return true;
}
}  // namespace

PathsConnector::PathsConnector(double pathLengthTolerance, Graph & graph,
                               RoadInfoGetter & infoGetter, v2::Stats & stat)
  : m_pathLengthTolerance(pathLengthTolerance)
  , m_graph(graph)
  , m_infoGetter(infoGetter)
  , m_stat(stat)
{
}

bool PathsConnector::ConnectCandidates(vector<LocationReferencePoint> const & points,
                                       vector<vector<ScorePath>> const & lineCandidates,
                                       vector<Graph::EdgeVector> & resultPath)
{
  // @TODO resultPath should be vector<ScorePathVec> and a comment should be added
  // about content of resultPath.
  ASSERT(!points.empty(), ());

  resultPath.resize(points.size() - 1);

  // TODO(mgsergio): Discard last point on failure.
  // TODO(mgsergio): Do not iterate more than kMaxRetries times.
  // TODO(mgserigio): Make kMaxRetries depend on points number in the segment.
//  LOG(LINFO, ("ConnectCandidates() points.size()", points.size()));
  for (size_t i = 1; i < points.size(); ++i)
  {
//    LOG(LINFO, ("ConnectCandidates() next pnt"));
//    bool found = false;

    auto const & point = points[i - 1];
    auto const distanceToNextPoint = static_cast<double>(point.m_distanceToNextPoint);
    auto const & fromCandidates = lineCandidates[i - 1];
    auto const & toCandidates = lineCandidates[i];
    auto & resultPathPart = resultPath[i - 1];

//    Graph::EdgeVector fakePath;
    vector<ScorePath> result;
    for (size_t fromInd = 0; fromInd < fromCandidates.size(); ++fromInd)
    {
      for (size_t toInd = 0; toInd < toCandidates.size(); ++toInd)
      {
//        LOG(LINFO, ("fromCandidates[", fromInd, "]:", fromCandidates[fromInd].m_score,
//                    "toCandidates[", toInd, "]", toCandidates[toInd].m_score));
//        resultPathPart.clear();

        Graph::EdgeVector path;
        if (!ConnectAdjacentCandidateLines(fromCandidates[fromInd].m_path,
                                           toCandidates[toInd].m_path, point.m_lfrcnp,
                                           distanceToNextPoint, path))
        {
          continue;
        }

        // @TODO Edit score
        // @TODO Continue if found
        // @TODO fromCandidates should be only if there's successful route to toCandidates of
        //  preview route.
        Score2 score = ValidatePath(path, distanceToNextPoint, m_pathLengthTolerance);

        // Checking for uniformity
//        ftypes::HighwayClass hwClass = ftypes::HighwayClass::Undefined;
        ftypes::HighwayClass minHwClass = ftypes::HighwayClass::Undefined;
        ftypes::HighwayClass maxHwClass = ftypes::HighwayClass::Undefined;
//        bool hwClassIsTheSame = true;
        bool oneWay = false;
        bool oneWayIsTheSame = true;
        bool roundabout = false;
        bool roundaboutIsTheSame = true;
        bool link = false;
        bool linkIsTheSame = true;
        for (auto const & p : path)
        {
          if (p.IsFake())
            continue;

          feature::TypesHolder types;
          m_graph.GetFeatureTypes(p.GetFeatureId(), types);
          if (minHwClass == ftypes::HighwayClass::Undefined)
          {
            minHwClass = ftypes::GetHighwayClass(types);
            maxHwClass = minHwClass;
            oneWay = ftypes::IsOneWayChecker::Instance()(types);
            roundabout = ftypes::IsRoundAboutChecker::Instance()(types);
            link = ftypes::IsLinkChecker::Instance()(types);
          }
          else
          {
            ftypes::HighwayClass hwClass = ftypes::GetHighwayClass(types);
            minHwClass = static_cast<ftypes::HighwayClass>(
                min(static_cast<uint8_t>(minHwClass), static_cast<uint8_t>(hwClass)));
            maxHwClass = static_cast<ftypes::HighwayClass>(
                max(static_cast<uint8_t>(maxHwClass), static_cast<uint8_t>(hwClass)));

//            if (hwClassIsTheSame && hwClass != ftypes::GetHighwayClass(types))
//              hwClassIsTheSame = false;
            if (oneWayIsTheSame && oneWay != ftypes::IsOneWayChecker::Instance()(types))
              oneWayIsTheSame = false;
            if (roundaboutIsTheSame && roundabout != ftypes::IsRoundAboutChecker::Instance()(types))
              roundaboutIsTheSame = false;
            if (linkIsTheSame && link != ftypes::IsLinkChecker::Instance()(types))
              linkIsTheSame = false;
          }
        }

        uint8_t const hwClassDiff = static_cast<uint8_t>(maxHwClass) - static_cast<uint8_t>(minHwClass);

        // @TODO Use the diff better:
        // score = static_cast<Score2>(score * (hwClassDiff == 0 ? 1.1 : hwClassDiff == 1 ? 1.0 : 0.9));
        score = static_cast<Score2>(score * (hwClassDiff == 0 ? 1.1 : hwClassDiff == 1 ? 1.05 : 1.0));

        if (minHwClass != ftypes::HighwayClass::Undefined && oneWayIsTheSame && roundaboutIsTheSame && linkIsTheSame)
          score = static_cast<Score2>(score * 1.15);

        if (path.front().IsFake() || path.back().IsFake())
          score = 0;
        result.emplace_back(score + fromCandidates[fromInd].m_score + toCandidates[toInd].m_score, move(path));
//        LOG(LINFO, ("score:", result.back().m_score));
      }
    }

//    if (!fakePath.empty() && !found)
//    {
//      found = true;
//      resultPathPart = fakePath;
//    }

    if (result.empty())
    {
      LOG(LDEBUG, ("No shortest path found"));
      ++m_stat.m_noShortestPathFound;
      resultPathPart.clear();
      return false;
    }

    auto const it = std::max_element(result.cbegin(), result.cend(),
                                     [](ScorePath const & o1, ScorePath const & o2) {
                                       return o1.m_score < o2.m_score;
                                     });
    resultPathPart = it->m_path;
    LOG(LINFO, ("best score:", it->m_score));
  }

  ASSERT_EQUAL(resultPath.size(), points.size() - 1, ());

  return true;
}

bool PathsConnector::FindShortestPath(Graph::Edge const & from, Graph::Edge const & to,
                                      FunctionalRoadClass lowestFrcToNextPoint, uint32_t maxPathLength,
                                      Graph::EdgeVector & path)
{
  // TODO(mgsergio): Turn Dijkstra to A*.
  uint32_t const kLengthToleranceM = 10;

  struct State
  {
    State(Graph::Edge const & e, uint32_t const s) : m_edge(e), m_score(s) {}

    bool operator>(State const & o) const
    {
      return make_tuple(m_score, m_edge) > make_tuple(o.m_score, o.m_edge);
    }

    Graph::Edge m_edge;
    uint32_t m_score;
  };

  ASSERT(from.HasRealPart() && to.HasRealPart(), ());

  priority_queue<State, vector<State>, greater<State>> q;
  map<Graph::Edge, uint32_t> scores;
  map<Graph::Edge, Graph::Edge> links;

  q.emplace(from, 0);
  scores[from] = 0;

  while (!q.empty())
  {
    auto const state = q.top();
    q.pop();

    auto const & u = state.m_edge;
    // TODO(mgsergio): Unify names: use either score or distance.
    auto const us = state.m_score;

    if (us > maxPathLength + kLengthToleranceM)
      continue;

    if (us > scores[u])
      continue;

    if (u == to)
    {
      for (auto e = u; e != from; e = links[e])
        path.push_back(e);
      path.push_back(from);
      reverse(begin(path), end(path));
      return true;
    }

    Graph::EdgeVector edges;
    m_graph.GetOutgoingEdges(u.GetEndJunction(), edges);
    for (auto const & e : edges)
    {
      if (!ConformLfrcnp(e, lowestFrcToNextPoint, m_infoGetter))
        continue;
      // TODO(mgsergio): Use frc to filter edges.

      // Only start and/or end of the route can be fake.
      // Routes made only of fake edges are no used to us.
      if (u.IsFake() && e.IsFake())
        continue;

      auto const it = scores.find(e);
      auto const eScore = us + EdgeLength(e);
      if (it == end(scores) || it->second > eScore)
      {
        scores[e] = eScore;
        links[e] = u;
        q.emplace(e, eScore);
      }
    }
  }

  return false;
}

bool PathsConnector::ConnectAdjacentCandidateLines(Graph::EdgeVector const & from,
                                                   Graph::EdgeVector const & to,
                                                   FunctionalRoadClass lowestFrcToNextPoint,
                                                   double distanceToNextPoint,
                                                   Graph::EdgeVector & resultPath)

{
  ASSERT(!to.empty(), ());

  if (auto const skip = PathOverlappingLen(from, to))
  {
    if (skip == -1)
      return false;
    copy(begin(from), end(from), back_inserter(resultPath));
    copy(begin(to) + skip, end(to), back_inserter(resultPath));
    return true;
  }

  ASSERT(from.back() != to.front(), ());

  Graph::EdgeVector shortestPath;
  auto const found =
      FindShortestPath(from.back(), to.front(), lowestFrcToNextPoint, distanceToNextPoint, shortestPath);
  if (!found)
    return false;

  // Skip the last edge from |from| because it already took its place at begin(shortestPath).
  copy(begin(from), prev(end(from)), back_inserter(resultPath));
  copy(begin(shortestPath), end(shortestPath), back_inserter(resultPath));
  // Skip the first edge from |to| because it already took its place at prev(end(shortestPath)).
  copy(next(begin(to)), end(to), back_inserter(resultPath));

  return found;
}
}  // namespace openlr
