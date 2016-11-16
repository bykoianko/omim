#pragma once

#include "routing/joint.hpp"
#include "routing/routing_serialization.hpp"

#include "coding/reader.hpp"
#include "coding/write_to_sink.hpp"

#include "std/cstdint.hpp"
#include "std/unordered_map.hpp"
#include "std/utility.hpp"
#include "std/vector.hpp"

namespace routing
{
class RoadJointIds final
{
public:
  Joint::Id GetJointId(uint32_t segId) const
  {
    if (segId < m_jointIds.size())
      return m_jointIds[segId];

    return Joint::kInvalidId;
  }

  void AddJoint(uint32_t segId, Joint::Id jointId)
  {
    if (segId >= m_jointIds.size())
      m_jointIds.insert(m_jointIds.end(), segId + 1 - m_jointIds.size(), Joint::kInvalidId);

    ASSERT_EQUAL(m_jointIds[segId], Joint::kInvalidId, ());
    m_jointIds[segId] = jointId;
  }

  template <typename F>
  void ForEachJoint(F && f) const
  {
    for (uint32_t segId = 0; segId < m_jointIds.size(); ++segId)
    {
      Joint::Id const jointId = m_jointIds[segId];
      if (jointId != Joint::kInvalidId)
        f(segId, jointId);
    }
  }

  size_t GetSize() const { return m_jointIds.size(); }

  Joint::Id Front() const { return m_jointIds.front(); }

  Joint::Id Back() const { return m_jointIds.back(); }

  template <class TSink>
  void Serialize(TSink & sink) const
  {
    WriteToSink(sink, static_cast<Joint::Id>(m_jointIds.size()));
    for (Joint::Id jointId : m_jointIds)
      WriteToSink(sink, jointId);
  }

  template <class TSource>
  void Deserialize(TSource & src)
  {
    Joint::Id const jointsSize = ReadPrimitiveFromSource<Joint::Id>(src);
    m_jointIds.reserve(jointsSize);
    for (Joint::Id i = 0; i < jointsSize; ++i)
    {
      Joint::Id const jointId = ReadPrimitiveFromSource<Joint::Id>(src);
      m_jointIds.emplace_back(jointId);
    }
  }

private:
  // Joint ids indexed by segment id.
  // If some segment id doesn't match any joint id, this vector contains Joint::kInvalidId.
  vector<Joint::Id> m_jointIds;
};

class RoadIndex final
{
public:
  void Import(vector<Joint> const & joints);

  /// \brief if |featureIdFrom| and |featureIdTo| are adjacent and if they are connected by
  /// its ends fills |from| and |to| with corresponding |from| and |to| and return true.
  /// If not returns false.
  bool GetAdjacentFtPoints(uint32_t featureIdFrom, uint32_t featureIdTo,
                           RoadPoint & from, RoadPoint & to, Joint::Id & jointId) const;

  void AddJoint(RoadPoint rp, Joint::Id jointId)
  {
    m_roads[rp.GetFeatureId()].AddJoint(rp.GetPointId(), jointId);
  }

  // Find nearest point with normal joint id.
  // If forward == true: neighbor with larger point id (right neighbor)
  // If forward == false: neighbor with smaller point id (left neighbor)
  pair<Joint::Id, uint32_t> FindNeighbor(RoadPoint rp, bool forward) const;

  template <class Sink>
  void Serialize(Sink & sink) const
  {
    WriteToSink(sink, static_cast<uint32_t>(m_roads.size()));
    for (auto const & it : m_roads)
    {
      uint32_t const featureId = it.first;
      WriteToSink(sink, featureId);
      it.second.Serialize(sink);
    }
  }

  template <class Source>
  void Deserialize(Source & src)
  {
    size_t const roadsSize = static_cast<size_t>(ReadPrimitiveFromSource<uint32_t>(src));
    for (size_t i = 0; i < roadsSize; ++i)
    {
      uint32_t featureId = ReadPrimitiveFromSource<decltype(featureId)>(src);
      m_roads[featureId].Deserialize(src);
    }
  }

  uint32_t GetSize() const { return m_roads.size(); }

  Joint::Id GetJointId(RoadPoint rp) const
  {
    auto const it = m_roads.find(rp.GetFeatureId());
    if (it == m_roads.end())
      return Joint::kInvalidId;

    return it->second.GetJointId(rp.GetPointId());
  }

  template <typename F>
  void ForEachRoad(F && f) const
  {
    for (auto const & it : m_roads)
      f(it.first, it.second);
  }

  template <typename F>
  void ForEachJoint(uint32_t featureId, F && f) const
  {
    auto const it = m_roads.find(featureId);
    if (it == m_roads.cend())
      return;

    it->second.ForEachJoint(f);
  }

private:
  // Map from feature id to RoadJointIds.
  unordered_map<uint32_t, RoadJointIds> m_roads;
};
}  // namespace routing
