#pragma once

#include "platform/country_file.hpp"

#include "base/assert.hpp"
#include "base/checked_cast.hpp"

#include <cstdint>
#include <map>
#include <vector>

namespace storage
{
using NumMwmId = std::uint16_t;

class NumMwmIds final
{
public:
  void RegisterFile(platform::CountryFile const & file)
  {
    if (m_fileToId.find(file) != m_fileToId.cend())
      return;

    NumMwmId const id = base::asserted_cast<NumMwmId>(m_idToFile.size());
    m_idToFile.push_back(file);
    m_fileToId[file] = id;
  }

  platform::CountryFile const & GetFile(NumMwmId mwmId) const
  {
    size_t const index = base::asserted_cast<size_t>(mwmId);
    CHECK_LESS(index, m_idToFile.size(), ());
    return m_idToFile[index];
  }

  NumMwmId GetId(platform::CountryFile const & file) const
  {
    auto const it = m_fileToId.find(file);
    CHECK(it != m_fileToId.cend(), ("Can't find mwm id for", file));
    return it->second;
  }

private:
  std::vector<platform::CountryFile> m_idToFile;
  std::map<platform::CountryFile, NumMwmId> m_fileToId;
};
}  //  namespace storage
