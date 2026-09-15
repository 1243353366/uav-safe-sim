// uav-safe-sim - Go telemetry relay (REFERENCE component - not yet compiled)
//
// Language rationale: Go is assigned to the networking edge because the
// telemetry relay is a stateless fan-out service, and Go's static binaries
// and goroutine model suit a small, deployable network component. It is
// explicitly NOT assigned to anything in the control loop.
//
// Status: REFERENCE - not compiled here (no Go toolchain in CI yet).
// The provenance schema it forwards is defined in src/uav/provenance.py;
// compatibility is NOT assumed and must be verified by schema tests.
package main

import (
	"bufio"
	"encoding/json"
	"flag"
	"fmt"
	"net"
	"os"
	"sync"
)

// DecisionRecord mirrors src/uav/provenance.py DecisionRecord.as_dict().
// NOTE: this struct is a *candidate* mirror. Whether the Go struct stays
// in sync with the Python dataclass is exactly the kind of cross-language
// drift this experiment measures. The authoritative schema is the Python
// provenance module + the SQL schema, not this struct.
type DecisionRecord struct {
	T            string          `json:"t"`
	Component    string          `json:"component"`
	Decision     string          `json:"decision"`
	Inputs       json.RawMessage `json:"inputs"`
	Uncertainty  *float64        `json:"uncertainty"`
	ModelVersion string          `json:"model_version"`
	Consumer     *string         `json:"consumer"`
	Rationale    *string         `json:"rationale"`
}

// validate enforces the observability contract at the relay boundary:
// a record without provenance fields is dropped and counted, never forwarded.
func validate(rec *DecisionRecord) error {
	if rec.Component == "" {
		return fmt.Errorf("missing component")
	}
	if rec.Decision == "" {
		return fmt.Errorf("missing decision")
	}
	if rec.ModelVersion == "" {
		return fmt.Errorf("missing model_version (provenance is mandatory)")
	}
	return nil
}

func main() {
	addr := flag.String("listen", ":9400", "TCP listen address for fan-out")
	flag.Parse()

	// sinks that connected peers register with (fan-out set)
	var mu sync.Mutex
	sinks := make(map[net.Conn]bool)

	ln, err := net.Listen("tcp", *addr)
	if err != nil {
		fmt.Fprintln(os.Stderr, "listen:", err)
		os.Exit(1)
	}
	go func() { // accept sink registrations
		for {
			conn, err := ln.Accept()
			if err != nil {
				return
			}
			mu.Lock()
			sinks[conn] = true
			mu.Unlock()
		}
	}()

	scanner := bufio.NewScanner(os.Stdin)
	scanner.Buffer(make([]byte, 1024*1024), 1024*1024)
	dropped := 0
	forwarded := 0
	for scanner.Scan() {
		line := scanner.Bytes()
		var rec DecisionRecord
		if err := json.Unmarshal(line, &rec); err != nil {
			dropped++
			continue
		}
		if err := validate(&rec); err != nil {
			dropped++
			continue
		}
		b, _ := json.Marshal(&rec)
		mu.Lock()
		for conn := range sinks {
			if _, err := conn.Write(append(b, '\n')); err != nil {
				delete(sinks, conn)
			}
		}
		mu.Unlock()
		forwarded++
	}
	fmt.Printf("relay: forwarded=%d dropped=%d\n", forwarded, dropped)
}
