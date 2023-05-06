package main

import (
	"errors"
	"fmt"
	"sync"
	"time"
)

type SnowFlake struct {
	sync.Mutex

	startTime     uint
	timestampBits uint
	dataIDBits    uint
	workerIDBits  uint
	sequenceBits  uint
	sequence      uint

	dataID   uint
	workerID uint

	timestampShift uint
	sequenceMask   uint
	dataIDValue    uint
	workerIDValue  uint

	lastTimestamp uint
}

func NewSnowFlake(startTime, timestampBits, dataIDBits, workerIDBits, sequenceBits, sequence, dataID, workerID uint) *SnowFlake {

	workerIDValue := workerID << sequenceBits
	dataIDValue := dataID << (sequenceBits + workerIDBits)
	timestampShift := sequenceBits + workerIDBits + dataIDBits

	sequenceMask := -1 ^ (-1 << sequenceBits)

	maxDataID := -1 ^ (-1 << dataIDBits)
	maxWorkerID := -1 ^ (-1 << workerIDBits)
	if dataID > uint(maxDataID) || dataID < 0 {
		panic(fmt.Sprintf("dataID %d 越界 0 %d", dataID, maxDataID))
	}
	if workerID > uint(maxWorkerID) || workerID < 0 {
		panic(fmt.Sprintf("workerID %d 越界 0 %d", workerID, maxWorkerID))
	}

	return &SnowFlake{
		startTime:     startTime,
		timestampBits: timestampBits,
		dataIDBits:    dataIDBits,
		workerIDBits:  workerIDBits,
		sequenceBits:  sequenceBits,
		sequence:      sequence,
		dataID:        dataID,
		workerID:      workerID,

		timestampShift: timestampShift,
		sequenceMask:   uint(sequenceMask),
		dataIDValue:    dataIDValue,
		workerIDValue:  workerIDValue,

		lastTimestamp: 0,
	}
}

// 生成整型时间戳(毫秒)
func (s *SnowFlake) genTimestamp() uint {
	return uint(time.Now().UnixMilli())
}

// 等到下一毫秒
func (s *SnowFlake) getNextMilTime() uint {

	timestamp := s.genTimestamp()

	for timestamp <= s.lastTimestamp {
		timestamp = s.genTimestamp()
	}

	return timestamp
}

func (s *SnowFlake) GenID() (uint, error) {

	s.Lock()

	now := s.genTimestamp()

	if now < s.lastTimestamp {
		s.Unlock()
		return 0, errors.New("时钟回拨")
	}
	if now == s.lastTimestamp {
		s.sequence = (s.sequence + 1) & s.sequenceMask
		if s.sequence == 0 {
			now = s.getNextMilTime()
		}
	} else {
		s.sequence = 0
	}

	s.lastTimestamp = now
	timestampValue := (now - s.startTime) << s.timestampShift
	value := timestampValue | s.dataIDValue | s.workerIDValue | s.sequence

	s.Unlock()
	return value, nil
}

func main() {

	// "2023-01-02T15:04:05+08:00",

	now := time.Now()

	s := NewSnowFlake(
		uint(now.UnixMilli()),
		41,
		5,
		5,
		12,
		0,
		0,
		0,
	)

	for i := 0; i <= 10; i++ {
		// _, err := s.GenID()
		// if err != nil {
		// 	fmt.Println(err)
		// }
		value, err := s.GenID()
		if err != nil {
			fmt.Println(err)
		} else {
			fmt.Println(value)
		}
	}

	fmt.Println(time.Now().UnixMilli() - now.UnixMilli())
}
