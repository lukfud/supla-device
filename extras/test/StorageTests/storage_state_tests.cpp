/*
   Copyright (C) AC SOFTWARE SP. Z O.O

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   as published by the Free Software Foundation; either version 2
   of the License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
   */

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <supla/storage/storage.h>
#include <storage_mock.h>
#include <string.h>
#include <stdio.h>
#include <supla/element.h>

class ElementWithStorage : public Supla::Element {
 public:
  ElementWithStorage() {
  }

  void onLoadState() {
    Supla::Storage::ReadState(reinterpret_cast<unsigned char *>(&stateValue),
                              sizeof(stateValue));
  }

  void onSaveState() {
    Supla::Storage::WriteState(reinterpret_cast<unsigned char *>(&stateValue),
                               sizeof(stateValue));
  }

  int32_t stateValue = -1;
};

TEST(StorageStateTests, preambleInitialization) {
  EXPECT_FALSE(Supla::Storage::Init());

  StorageMockSimulator storage;

  Supla::Preamble preamble;
  memcpy(preamble.suplaTag, "SUPLA", 5);
  preamble.version = 1;
  preamble.sectionsCount = 0;

  EXPECT_CALL(storage, commit()).Times(1);

  EXPECT_TRUE(Supla::Storage::Init());

  EXPECT_EQ(memcmp(&preamble, storage.storageSimulatorData, 8), 0);
}

TEST(StorageStateTests, preambleAlreadyInitialized) {
  EXPECT_FALSE(Supla::Storage::Init());
  StorageMockSimulator storage;

  Supla::Preamble preamble;
  memcpy(preamble.suplaTag, "SUPLA", 5);
  preamble.version = 1;
  preamble.sectionsCount = 0;

  memcpy(storage.storageSimulatorData, &preamble, 8);

  EXPECT_CALL(storage, commit()).Times(0);

  EXPECT_TRUE(Supla::Storage::Init());
  EXPECT_EQ(memcmp(&preamble, storage.storageSimulatorData, 8), 0);
}

TEST(StorageStateTests, invalidPreambleAlreadyInitialized) {
  EXPECT_FALSE(Supla::Storage::Init());

  StorageMockSimulator storage;

  Supla::Preamble invalidPreamble;
  memcpy(invalidPreamble.suplaTag, "SuPLa", 5);
  invalidPreamble.version = 1;
  invalidPreamble.sectionsCount = 0;
  memcpy(storage.storageSimulatorData, &invalidPreamble, 8);

  Supla::Preamble preamble;
  memcpy(preamble.suplaTag, "SUPLA", 5);
  preamble.version = 1;
  preamble.sectionsCount = 0;

  EXPECT_CALL(storage, commit()).Times(1);

  EXPECT_TRUE(Supla::Storage::Init());
  EXPECT_EQ(memcmp(&preamble, storage.storageSimulatorData, 8), 0);
}


TEST(StorageStateTests, preambleInitializationWithElement) {
  EXPECT_FALSE(Supla::Storage::Init());

  StorageMockSimulator storage;
  ElementWithStorage el;

  Supla::Preamble preamble;
  memcpy(preamble.suplaTag, "SUPLA", 5);
  preamble.version = 1;
  preamble.sectionsCount = 0;

  memcpy(storage.storageSimulatorData, &preamble, 8);

  Supla::Preamble preambleWithStateSection;
  memcpy(preambleWithStateSection.suplaTag, "SUPLA", 5);
  preambleWithStateSection.version = 1;
  preambleWithStateSection.sectionsCount = 1;

  EXPECT_CALL(storage, commit()).Times(2);

  EXPECT_TRUE(Supla::Storage::Init());
  ASSERT_FALSE(Supla::Storage::IsStateStorageValid());
  Supla::Storage::WriteStateStorage();

  EXPECT_EQ(el.stateValue, -1);

  el.stateValue = 123456;

  Supla::Storage::LoadStateStorage();
  EXPECT_EQ(el.stateValue, -1);

  el.stateValue = 123456;
  Supla::Storage::WriteStateStorage();
  Supla::Storage::LoadStateStorage();
  EXPECT_EQ(el.stateValue, 123456);


  EXPECT_EQ(memcmp(&preambleWithStateSection, storage.storageSimulatorData, 8),
            0);

  Supla::SectionPreamble secPreamble = {};
  secPreamble.type = STORAGE_SECTION_TYPE_ELEMENT_STATE;
  secPreamble.size = 4;
  secPreamble.crc1 = 17076;
  secPreamble.crc2 = 17076;
  EXPECT_EQ(memcmp(&secPreamble, storage.storageSimulatorData + 8, 7), 0);
}

TEST(StorageStateTests, preambleAlreadyInitializedWithElement) {
  EXPECT_FALSE(Supla::Storage::Init());

  StorageMockSimulator storage;
  ElementWithStorage el;

  Supla::Preamble preamble;
  memcpy(preamble.suplaTag, "SUPLA", 5);
  preamble.version = 1;
  preamble.sectionsCount = 1;

  memcpy(storage.storageSimulatorData, &preamble, 8);

  Supla::SectionPreamble secPreamble = {};
  secPreamble.type = STORAGE_SECTION_TYPE_ELEMENT_STATE;
  secPreamble.size = 4;
  secPreamble.crc1 = 17076;
  secPreamble.crc2 = 17076;

  memcpy(storage.storageSimulatorData + 8, &secPreamble, 7);

  int32_t valueInStorage = 123456;
  memcpy(storage.storageSimulatorData + 15, &valueInStorage, 4);

  EXPECT_CALL(storage, commit()).Times(2);

  EXPECT_TRUE(Supla::Storage::Init());
  ASSERT_TRUE(Supla::Storage::IsStateStorageValid());

  EXPECT_EQ(el.stateValue, -1);
  Supla::Storage::LoadStateStorage();
  EXPECT_EQ(el.stateValue, 123456);

  storage.noWriteExpected = true;
  // No change in state -> no write operations
  Supla::Storage::WriteStateStorage();

  // Change state and update storage
  el.stateValue = 44;
  storage.noWriteExpected = false;
  Supla::Storage::WriteStateStorage();
  EXPECT_EQ(el.stateValue, 44);
  valueInStorage = 44;
  EXPECT_EQ(memcmp(storage.storageSimulatorData + 15, &valueInStorage, 4), 0);
}

