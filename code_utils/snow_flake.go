package main

import (
	"errors"
	"fmt"
	"sync"
	"time"
)

var (
	idWorkerMap = make(map[string]*IdWorker)

	keys = [...]string{"users"}
)

func init() {

	start, err := time.Parse(time.RFC3339, "2023-05-08T01:01:01+08:00")
	if err != nil {
		panic(err)
	}

	for _, value := range keys {
		idWorkerMap[value] = newIDWorker(
			uint(start.UnixMilli()),
			41,
			5,
			5,
			12,
			0,
			0,
			0,
		)
	}
}

type IdWorker struct {
	sync.Mutex

	startTime     uint
	timestampBits uint
	dataIDBits    uint
	workerIDBits  uint
	sequenceBits  uint
	dataID        uint
	workerID      uint
	sequence      uint

	timestampShift uint
	sequenceMask   uint
	dataIDValue    uint
	workerIDValue  uint

	lastTimestamp uint
}

func newIDWorker(
	startTime, timestampBits, dataIDBits, workerIDBits, sequenceBits, dataID, workerID, sequence uint) *IdWorker {

	workerIDValue := workerID << sequenceBits
	dataIDValue := dataID << (sequenceBits + workerIDBits)
	timestampShift := sequenceBits + workerIDBits + dataIDBits

	sequenceMask := -1 ^ (-1 << sequenceBits)

	maxDataID := -1 ^ (-1 << dataIDBits)
	maxWorkerID := -1 ^ (-1 << workerIDBits)
	if dataID > uint(maxDataID) {
		panic(fmt.Sprintf("dataID %d 越界 0 %d", dataID, maxDataID))
	}
	if workerID > uint(maxWorkerID) {
		panic(fmt.Sprintf("workerID %d 越界 0 %d", workerID, maxWorkerID))
	}

	return &IdWorker{
		startTime:     startTime,
		timestampBits: timestampBits,
		dataIDBits:    dataIDBits,
		workerIDBits:  workerIDBits,
		sequenceBits:  sequenceBits,
		dataID:        dataID,
		workerID:      workerID,
		sequence:      sequence,

		timestampShift: timestampShift,
		sequenceMask:   uint(sequenceMask),
		dataIDValue:    dataIDValue,
		workerIDValue:  workerIDValue,

		lastTimestamp: 0,
	}
}

// 生成整型时间戳(毫秒)
func (s *IdWorker) genTimestamp() uint {
	return uint(time.Now().UnixMilli())
}

// 等到下一毫秒
func (s *IdWorker) getNextMilTime() uint {

	timestamp := s.genTimestamp()

	for timestamp <= s.lastTimestamp {
		timestamp = s.genTimestamp()
	}

	return timestamp
}

func (s *IdWorker) genID() (uint, error) {

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

	// 跳过0
	if value == 0 {
		return s.genID()
	}

	return value, nil
}

// GetIDWorker 根据 key 获取指定的 idWorker 对象
func GetIDWorker(key string) (*IdWorker, error) {

	worker, ok := idWorkerMap[key]
	if !ok {
		return nil, errors.New("not ok, this key")
	}

	return worker, nil
}

// GetID 根据 key 获取指定的 idWorker 生成的 ID
func GetID(key string) (uint, error) {

	worker, err := GetIDWorker(key)
	if err != nil {
		return 0, err
	}

	return worker.genID()
}

func main() {

	fmt.Println("hello world")

	model, ok := idWorkerMap["users"]
	if !ok {
		fmt.Println("not ok, this key")
	}

	for i := 0; i <= 20; i++ {

		value, err := model.genID()
		if err != nil {
			fmt.Println(err)
		}
		fmt.Println(value)
	}
}

func TestIDWorker() {

	// "2023-01-02T15:04:05+08:00",

	now := time.Now()

	s := newIDWorker(
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
		// _, err := s.genID()
		// if err != nil {
		// 	fmt.Println(err)
		// }
		value, err := s.genID()
		if err != nil {
			fmt.Println(err)
		} else {
			fmt.Println(value)
		}
	}

	fmt.Println(time.Now().UnixMilli() - now.UnixMilli())
}
