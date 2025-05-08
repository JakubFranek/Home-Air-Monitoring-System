# Issues

This is a list of issues that have been observed during the project's deployment.

1. Node & hub communication sometimes hiccups and a few consecutive messages are not received by the hub
   - Affected nodes: all
   - Occurence: observed cca once per day
   - Suspect: nRF24 radio module (weak signal, low quality of Chinese knockoff modules), walls with metal rebars acting as radio reflectors
2. Some nodes experience very long term communication failures (duration of several hours or more)
    - Affected nodes: board no. 6 (balcony)
    - Occurence: observed once during the first three months of operation
    - Suspect: none (board no. 6 has been once again made operational for unknown reason)
3. Erroneous node message values are sometimes displayed
   - Temperatures of hundreds of degrees Celsius or relative humidities of hundreds of percents have been observed, as well as status codes in high tens or VDDAs of 70 Volts etc.
   - Often occurs at the same time as radio communication errors
   - There is no known source or sensible explanation for this as of yet... maybe extra CRC byte in nRF24 protocol would help?
   - Affected nodes: all
   - Occurence: observed cca once per month
   - Suspect: none