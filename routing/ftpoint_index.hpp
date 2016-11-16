#pragma once

#include "routing/joint.hpp"

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

class FtPointIndex final
{
public:
  void Import(vector<Joint> const & joints);

  void AddJoint(FtPoint ftp, Joint::Id jointId)
  {
    m_roads[ftp.GetFeatureId()].AddJoint(ftp.GetPointId(), jointId);
  }

  // Find nearest point with normal joint id.
  // If forward == true: neighbor with larger point id (right neighbor)
  // If forward == false: neighbor with smaller point id (left neighbor)
  pair<Joint::Id, uint32_t> FindNeighbor(FtPoint ftp, bool forward) const;

  template <class TSink>
  void Serialize(TSink & sink) const
  {
    WriteToSink(sink, static_cast<uint32_t>(m_roads.size()));
    for (auto const & it : m_roads)
    {
      uint32_t const featureId = it.first;
      WriteToSink(sink, featureId);
      it.second.Serialize(sink);
    }
  }

  template <class TSource>
  void Deserialize(TSource & src)
  {
    size_t const roadsSize = static_cast<size_t>(ReadPrimitiveFromSource<uint32_t>(src));
    for (size_t i = 0; i < roadsSize; ++i)
    {
      uint32_t featureId = ReadPrimitiveFromSource<decltype(featureId)>(src);
      m_roads[featureId].Deserialize(src);
    }
  }

  uint32_t GetSize() const { return m_roads.size(); }

  Joint::Id GetJointId(FtPoint ftp) const
  {
    auto const it = m_roads.find(ftp.GetFeatureId());
    if (it == m_roads.end())
      return Joint::kInvalidId;

    return it->second.GetJointId(ftp.GetPointId());
  }

  template <typename F>
  void ForEachRoad(F && f) const
  {
    for (auto const & it : m_roads)
      f(it.first, it.second);
  }

private:
  // Map from feature id to RoadJointIds.
  unordered_map<uint32_t, RoadJointIds> m_roads;
};
}  // namespace routing
