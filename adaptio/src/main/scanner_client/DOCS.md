# Diagram

```plantuml
left to right direction

package scanner {
  class "ScannerClientImpl" as scanner::ScannerClientImpl
  interface "ScannerClient" as scanner::ScannerClient
  interface "ScannerObserver" as scanner::ScannerObserver

  scanner::ScannerClientImpl --> scanner::ScannerObserver
  scanner::ScannerClientImpl --|>  scanner::ScannerClient
}

```
