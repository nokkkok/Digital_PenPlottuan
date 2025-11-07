LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY sd_reader_top IS
    PORT (
        -- Clock and Reset
        i_clk            : IN  STD_LOGIC; -- 20MHz clock (p123)
        i_rst            : IN  STD_LOGIC; -- Active-high reset (p47)

        -- User Input
        i_btn_next       : IN  STD_LOGIC; -- "Next Command" button (p48)
        i_uart_rxd       : IN  STD_LOGIC; -- UART Receive Data
        
        -- Status LEDs
        o_heartbeat_led  : OUT STD_LOGIC;
        o_init_success_led : OUT STD_LOGIC; -- '1' when SD init is done
        o_file_found_led : OUT STD_LOGIC; -- '1' when command block is loaded
        
        -- 7-Segment Display
        o_seg_out        : OUT STD_LOGIC_VECTOR(6 DOWNTO 0);
        o_anode_out      : OUT STD_LOGIC_VECTOR(3 DOWNTO 0);

        -- SD Card SPI
        o_spi_cs         : OUT STD_LOGIC;
        o_spi_sclk       : OUT STD_LOGIC;
        o_spi_mosi       : OUT STD_LOGIC;
        i_spi_miso       : IN  STD_LOGIC;
        
        -- UART
        o_uart_txd       : OUT STD_LOGIC  -- UART Transmit Data
    );
END sd_reader_top;

ARCHITECTURE rtl OF sd_reader_top IS

    -- ****************************************************
    -- * This is the LBA (Block Address) of your first command
    -- * 2000 (decimal) = 0x07D0 (hex)
    -- ****************************************************
    CONSTANT C_START_LBA : STD_LOGIC_VECTOR(31 DOWNTO 0) := x"000007D0";

    -- Component for the 7-segment multiplexer
    COMPONENT seven_seg_mux IS
        GENERIC (
            G_CLK_FREQ_HZ     : INTEGER := 20_000_000;
            G_REFRESH_RATE_HZ : INTEGER := 800
        );
        PORT (
            i_clk       : IN  STD_LOGIC;
            i_rst       : IN  STD_LOGIC;
            i_hex_data  : IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
            o_seg       : OUT STD_LOGIC_VECTOR(6 DOWNTO 0);
            o_an        : OUT STD_LOGIC_VECTOR(3 DOWNTO 0)
        );
    END COMPONENT;

    -- Component for the Heartbeat LED
    COMPONENT heartbeat IS
        GENERIC ( G_CLK_FREQ_HZ : INTEGER := 20_000_000 );
        PORT (
            i_clk       : IN  STD_LOGIC;
            i_rst       : IN  STD_LOGIC;
            o_led       : OUT STD_LOGIC
        );
    END COMPONENT;
    
    -- Component for the SD Card Controller
    COMPONENT sd_controller IS
        GENERIC (
            clockRate : integer;
            slowClockDivider : integer;
            R1_TIMEOUT : integer;
            WRITE_TIMEOUT : integer
        );
        PORT (
            cs : out std_logic;
            mosi : out std_logic;
            miso : in std_logic;
            sclk : out std_logic;
            card_present : in std_logic;
            card_write_prot : in std_logic;
            rd : in std_logic;
            rd_multiple : in std_logic;
            dout : out std_logic_vector(7 downto 0);
            dout_avail : out std_logic;
            dout_taken : in std_logic;
            wr : in std_logic;
            wr_multiple : in std_logic;
            din : in std_logic_vector(7 downto 0);
            din_valid : in std_logic;
            din_taken : out std_logic;
            addr : in std_logic_vector(31 downto 0);
            erase_count : in std_logic_vector(7 downto 0);
            sd_error : out std_logic;
            sd_busy : out std_logic;
            sd_error_code : out std_logic_vector(2 downto 0);
            reset : in std_logic;
            clk : in std_logic;
            sd_type : out std_logic_vector(1 downto 0);
            sd_fsm : out std_logic_vector(7 downto 0)
        );
    END COMPONENT;

    -- Component for your UART Transmitter
    COMPONENT uart_tx IS
      GENERIC (
        CLK_FREQ  : integer := 20_000_000;
        BAUD_RATE : integer := 9600
      );
      PORT (
        CLK       : IN  STD_LOGIC;
        RST       : IN  STD_LOGIC;
        DIN       : IN  STD_LOGIC_VECTOR(7 DOWNTO 0);
        DIN_VLD   : IN  STD_LOGIC;
        DIN_RDY   : OUT STD_LOGIC;
        UART_TXD  : OUT STD_LOGIC
      );
    END COMPONENT;
    
    -- Component for your UART Receiver
    COMPONENT uart_rx IS
      GENERIC (
        CLK_FREQ  : integer := 20_000_000;
        BAUD_RATE : integer := 9600
      );
      PORT (
        clk     : IN  STD_LOGIC;
        rst     : IN  STD_LOGIC;
        rx      : IN  STD_LOGIC;
        rx_data : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
        rx_dv   : OUT STD_LOGIC
      );
    END COMPONENT;


    -- Define a 512-byte Block RAM type
    CONSTANT C_BRAM_DEPTH : INTEGER := 512;
    TYPE t_bram IS ARRAY (0 TO C_BRAM_DEPTH - 1) OF STD_LOGIC_VECTOR(7 DOWNTO 0);
    SIGNAL g_bram : t_bram := (OTHERS => (OTHERS => '0'));
    
    -- Define a line buffer for UART
    CONSTANT C_LINE_BUFFER_DEPTH : INTEGER := 256; -- Max line length
    TYPE t_line_buffer IS ARRAY (0 TO C_LINE_BUFFER_DEPTH - 1) OF STD_LOGIC_VECTOR(7 DOWNTO 0);
    SIGNAL g_line_buffer : t_line_buffer := (OTHERS => (OTHERS => '0'));

    -- Wires to connect to sd_controller
    SIGNAL w_sd_busy       : STD_LOGIC;
    SIGNAL w_sd_error      : STD_LOGIC;
    SIGNAL w_sd_error_code : STD_LOGIC_VECTOR(2 DOWNTO 0);
    SIGNAL w_rd_trig       : STD_LOGIC := '0';
    SIGNAL w_addr          : STD_LOGIC_VECTOR(31 DOWNTO 0) := (OTHERS => '0');
    SIGNAL w_dout          : STD_LOGIC_VECTOR(7 DOWNTO 0);
    SIGNAL w_dout_avail    : STD_LOGIC;
    SIGNAL w_dout_taken    : STD_LOGIC := '0';
    
    -- Wires for UART Interface
    SIGNAL w_uart_din      : STD_LOGIC_VECTOR(7 DOWNTO 0);
    SIGNAL w_uart_din_vld  : STD_LOGIC;
    SIGNAL w_uart_din_rdy  : STD_LOGIC;
    
    -- This FSM now handles button presses, buffer refills, and UART sending
    TYPE t_wrapper_state IS (
        S_INIT_WAIT,
        S_READ_CMD_START,
        S_READ_CMD_DATA,
        S_READ_TIMEOUT_ERROR,
        S_UART_SCAN_START,
        S_UART_SCAN_BRAM,
        S_UART_SEND_LINE_START,
        S_UART_SEND_LINE_BYTE,
        S_UART_WAIT_FOR_PRESS      -- New Idle state
    );
    SIGNAL r_wrapper_state : t_wrapper_state := S_INIT_WAIT;
    SIGNAL r_bram_addr     : INTEGER RANGE 0 TO C_BRAM_DEPTH := 0; -- For SD write
    SIGNAL r_init_done     : STD_LOGIC := '0';
    SIGNAL r_cmd_loaded    : STD_LOGIC := '0';
    
    SIGNAL r_read_timeout  : INTEGER RANGE 0 TO 20_000_000 := 0; -- 1 second timeout
    
    -- New signals for UART line processing
    SIGNAL r_uart_bram_addr    : INTEGER RANGE 0 TO C_BRAM_DEPTH; -- For BRAM read
    SIGNAL r_line_buffer_index : INTEGER RANGE 0 TO C_LINE_BUFFER_DEPTH;
    SIGNAL r_uart_send_index   : INTEGER RANGE 0 TO C_LINE_BUFFER_DEPTH;
    SIGNAL r_found_stop_char   : STD_LOGIC := '0'; -- **NEW FLAG**
        
    -- Wires for user interface
    SIGNAL w_cmd_index      : UNSIGNED(15 DOWNTO 0) := (OTHERS => '0');
    SIGNAL w_data_byte      : STD_LOGIC_VECTOR(7 DOWNTO 0);
    SIGNAL w_display_data   : STD_LOGIC_VECTOR(15 DOWNTO 0);
    
    -- Button control signals
    SIGNAL r_btn_sync_chain : STD_LOGIC_VECTOR(2 DOWNTO 0) := "111";
    SIGNAL w_btn_stable     : STD_LOGIC;
    
    -- Delay counter for 1 second between sectors
    -- 20MHz clock = 20,000,000 cycles per second
    CONSTANT C_DELAY_1SEC : INTEGER := 20_000_000;
    SIGNAL r_delay_counter : INTEGER RANGE 0 TO C_DELAY_1SEC := 0;
    
    -- Debug display
    SIGNAL r_debug_display : STD_LOGIC_VECTOR(15 DOWNTO 0) := x"1011"; -- "1n1t"
    
BEGIN

    -- Instantiate Heartbeat
    inst_heartbeat : heartbeat
        GENERIC MAP ( G_CLK_FREQ_HZ => 20_000_000 )
        PORT MAP ( i_clk => i_clk, i_rst => i_rst, o_led => o_heartbeat_led );

    -- Instantiate 7-Segment Display Driver
    inst_7seg : seven_seg_mux
        GENERIC MAP (
            G_CLK_FREQ_HZ     => 20_000_000,
            G_REFRESH_RATE_HZ => 800
        )
        PORT MAP (
            i_clk       => i_clk,
            i_rst       => i_rst,
            i_hex_data  => w_display_data,
            o_seg       => o_seg_out,
            o_an        => o_anode_out
        );
        
    -- Instantiate the SD Card Controller
    inst_sd_controller : sd_controller
        GENERIC MAP (
            clockRate        => 20_000_000, 
            slowClockDivider => 25,
            R1_TIMEOUT       => 10,
            WRITE_TIMEOUT    => 500
        )
        PORT MAP (
            cs              => o_spi_cs,
            mosi            => o_spi_mosi,
            miso            => i_spi_miso,
            sclk            => o_spi_sclk,
            card_present    => '1', 
            card_write_prot => '0', 
            rd              => w_rd_trig,
            rd_multiple     => '0',
            addr            => w_addr,
            dout            => w_dout,
            dout_avail      => w_dout_avail,
            dout_taken      => w_dout_taken,
            wr              => '0',
            wr_multiple     => '0',
            din             => (OTHERS => '0'),
            din_valid       => '0',
            din_taken       => OPEN,
            erase_count     => (OTHERS => '0'),
            sd_error        => w_sd_error,
            sd_busy         => w_sd_busy,
            sd_error_code   => w_sd_error_code,
            reset           => i_rst,
            clk             => i_clk,
            sd_type         => OPEN,
            sd_fsm          => OPEN
        );

    -- Instantiate your UART Transmitter
    inst_uart_tx : uart_tx
        GENERIC MAP (
            CLK_FREQ  => 20_000_000,
            BAUD_RATE => 9600 -- Using the default from your file
        )
        PORT MAP (
            CLK       => i_clk,
            RST       => i_rst,
            DIN       => w_uart_din,
            DIN_VLD   => w_uart_din_vld,
            DIN_RDY   => w_uart_din_rdy,
            UART_TXD  => o_uart_txd
        );
        
    -- Instantiate your UART Receiver
    inst_uart_rx : uart_rx
        GENERIC MAP (
            CLK_FREQ  => 20_000_000,
            BAUD_RATE => 9600 -- Using the default from your file
        )
        PORT MAP (
            clk     => i_clk,
            rst     => i_rst,
            rx      => i_uart_rxd,
            rx_data => OPEN,  -- Not used in this logic
            rx_dv   => OPEN   -- Not used in this logic
        );


    -- Synchronizer for button input (debounce)
    -- ************************************************************
    -- Button = 0: Auto-continue to next sector
    -- Button = 1: Stop and wait
    -- ************************************************************
    PROCESS(i_clk)
    BEGIN
        IF rising_edge(i_clk) THEN
            IF i_rst = '1' THEN
                r_btn_sync_chain <= "111";
            ELSE
                r_btn_sync_chain <= r_btn_sync_chain(1 DOWNTO 0) & i_btn_next;
            END IF;
        END IF;
    END PROCESS;
    w_btn_stable <= r_btn_sync_chain(2);  -- Debounced button state

    -- Wrapper FSM: Handles Init, Read, UART Send, and Refill-on-Button-Press
    -- ** THIS PROCESS IS MODIFIED TO STOP AT 0x0A **
    PROCESS (i_clk)
    BEGIN
        IF rising_edge(i_clk) THEN
            IF i_rst = '1' THEN
                r_wrapper_state <= S_INIT_WAIT;
                w_rd_trig <= '0';
                w_dout_taken <= '0';
                r_bram_addr <= 0;
                r_init_done <= '0';
                r_cmd_loaded <= '0';
                r_read_timeout <= 0;
                w_addr <= (OTHERS => '0');
                r_debug_display <= x"1011"; -- "1n1t"
                w_cmd_index <= (OTHERS => '0');
                
                -- Reset UART signals
                w_uart_din_vld <= '0';
                w_uart_din <= (OTHERS => '0');
                r_uart_bram_addr <= 0;
                r_line_buffer_index <= 0;
                r_uart_send_index <= 0;
                r_found_stop_char <= '0'; -- Reset new flag
            ELSE
                -- Default assignments
                w_dout_taken <= '0';
                w_uart_din_vld <= '0'; -- Default to 0, only assert when sending
            
                CASE r_wrapper_state IS
                
                    -- State 1: Wait for SD card to initialize
                    WHEN S_INIT_WAIT =>
                        IF w_sd_busy = '0' THEN
                            r_init_done <= '1';
                            r_wrapper_state <= S_READ_CMD_START;
                            r_debug_display <= x"0A0F"; -- "rd F" (Read File)
                        END IF;
                        
                    -- State 2: Start reading block from (Start LBA + Command Index)
                    WHEN S_READ_CMD_START =>
                        w_rd_trig <= '1';
                        w_addr <= std_logic_vector(unsigned(C_START_LBA) + resize(w_cmd_index, 32));
                        r_bram_addr <= 0;
                        r_read_timeout <= 0; 
                        r_cmd_loaded <= '0'; -- Turn off LED during read
                        r_wrapper_state <= S_READ_CMD_DATA;
                        
                    -- State 3: Read File data into BRAM
                    WHEN S_READ_CMD_DATA =>
                        w_rd_trig <= '1'; 
                        
                        IF w_dout_avail = '1' THEN
                            g_bram(r_bram_addr) <= w_dout;
                            w_dout_taken <= '1'; 
                            r_read_timeout <= 0;
                            
                            IF r_bram_addr = (C_BRAM_DEPTH - 1) THEN
                                w_rd_trig <= '0'; -- Stop reading
                                r_cmd_loaded <= '1'; -- Turn on LED
                                r_wrapper_state <= S_UART_SCAN_START; -- Go to UART sending
                            ELSE
                                r_bram_addr <= r_bram_addr + 1;
                            END IF;
                        
                        ELSIF r_read_timeout = 20_000_000 THEN
                            w_rd_trig <= '0'; 
                            r_wrapper_state <= S_READ_TIMEOUT_ERROR;
                            r_debug_display <= x"E006"; -- Error 06 (File Timeout)
                        ELSE
                            r_read_timeout <= r_read_timeout + 1;
                        END IF;
                        
                    -- State 4: Start scanning the BRAM to send to UART
                    WHEN S_UART_SCAN_START =>
                        r_uart_bram_addr <= 0;
                        r_line_buffer_index <= 0;
                        w_uart_din_vld <= '0';
                        r_found_stop_char <= '0'; -- Reset flag for this new block
                        r_wrapper_state <= S_UART_SCAN_BRAM;
                        
                    -- State 5: Scan BRAM, buffer line
                    WHEN S_UART_SCAN_BRAM =>
                        -- Read byte from BRAM and store in line buffer
                        g_line_buffer(r_line_buffer_index) <= g_bram(r_uart_bram_addr);
                        
                        IF (g_bram(r_uart_bram_addr) = x"0A") THEN
                            -- **Found 0x0A (LF). This is the last line.**
                            r_found_stop_char <= '1'; -- Set flag
                            r_line_buffer_index <= r_line_buffer_index + 1; -- Store this last byte
                            r_wrapper_state <= S_UART_SEND_LINE_START;      -- Go send the line
                            
                        ELSIF (r_line_buffer_index = (C_LINE_BUFFER_DEPTH - 1)) THEN
                            -- Line buffer is full, send this line
                            r_line_buffer_index <= r_line_buffer_index + 1; 
                            r_wrapper_state <= S_UART_SEND_LINE_START;      
                            
                        ELSIF (r_uart_bram_addr = (C_BRAM_DEPTH - 1)) THEN
                            -- Reached end of BRAM, send this final partial line
                            r_line_buffer_index <= r_line_buffer_index + 1; 
                            r_wrapper_state <= S_UART_SEND_LINE_START;      
                            
                        ELSE
                            -- Just a normal byte, keep scanning
                            r_line_buffer_index <= r_line_buffer_index + 1;
                            r_uart_bram_addr <= r_uart_bram_addr + 1;
                            r_wrapper_state <= S_UART_SCAN_BRAM;
                        END IF;
                        
                    -- State 6: Start sending the buffered line
                    WHEN S_UART_SEND_LINE_START =>
                        r_uart_send_index <= 0;
                        r_wrapper_state <= S_UART_SEND_LINE_BYTE;
                        
                    -- State 7: Send one byte of the line (Corrected Handshake)
                    WHEN S_UART_SEND_LINE_BYTE =>
                        IF r_uart_send_index = r_line_buffer_index THEN
                            -- **DONE WITH THIS LINE**
                            r_line_buffer_index <= 0; -- Clear line buffer
                            
                            IF (r_found_stop_char = '1') THEN
                                -- **Found 0A, stop all sending for this block**
                                r_wrapper_state <= S_UART_WAIT_FOR_PRESS;
                                
                            ELSIF (r_uart_bram_addr = (C_BRAM_DEPTH - 1)) THEN
                                -- We were also at the end of the BRAM
                                r_wrapper_state <= S_UART_WAIT_FOR_PRESS;
                            ELSE
                                -- Not at end of BRAM, advance BRAM pointer and scan for next line
                                r_uart_bram_addr <= r_uart_bram_addr + 1;
                                r_wrapper_state <= S_UART_SCAN_BRAM;
                            END IF;
                        
                        ELSE
                            -- **STILL SENDING BYTES IN THIS LINE**
                            IF w_uart_din_rdy = '1' THEN
                                -- UART is ready for a byte. Send it.
                                w_uart_din <= g_line_buffer(r_uart_send_index);
                                w_uart_din_vld <= '1';
                                r_uart_send_index <= r_uart_send_index + 1; -- Increment index *after* sending
                            ELSE
                                -- UART is busy. Hold data and valid signal.
                                -- Keep `w_uart_din_vld` '0' (default) and wait.
                                -- The index is not incremented, so we will retry this same byte.
                            END IF;
                        END IF;
                        
                    -- State 9: BRAM sent. Check button to continue or wait.
                    WHEN S_UART_WAIT_FOR_PRESS =>
                        IF w_btn_stable = '0' THEN
                            -- Button held at 0: Auto-continue to next sector
                            IF r_delay_counter = C_DELAY_1SEC THEN
                                -- 1 second delay has elapsed
                                w_cmd_index <= w_cmd_index + 1;
                                r_wrapper_state <= S_READ_CMD_START;
                                r_debug_display <= x"0A0F"; -- "rd F"
                                r_delay_counter <= 0; -- Reset delay counter
                            ELSE
                                r_delay_counter <= r_delay_counter + 1;
                            END IF;
                        ELSE
                            -- Button = 1: Reset delay and stay in wait state
                            r_delay_counter <= 0;
                        END IF;
                        
                    -- State 10: Timeout Error
                    WHEN S_READ_TIMEOUT_ERROR =>
                        w_rd_trig <= '0';
                
                END CASE;
            END IF;
        END IF;
    END PROCESS;

    -- Read data from BRAM. We only show the first byte.
    w_data_byte <= g_bram(0);
    
    -- 7-Segment Display Mux
    PROCESS(w_sd_error, w_sd_error_code, r_init_done, w_cmd_index, w_data_byte, r_wrapper_state, r_debug_display, r_cmd_loaded)
    BEGIN
        IF w_sd_error = '1' THEN
            -- Show "EE0X" (SD Controller Error)
            w_display_data <= x"EE0" & '0' & w_sd_error_code;
        ELSIF r_wrapper_state = S_READ_TIMEOUT_ERROR THEN
            -- Show the internal error code (e.g., "E006")
            w_display_data <= r_debug_display;
        ELSIF r_cmd_loaded = '1' THEN
            -- SUCCESS! Show Command Index (first 2 digits) and First Byte of Data (last 2 digits)
            w_display_data <= std_logic_vector(resize(w_cmd_index(7 DOWNTO 0), 8)) & w_data_byte;
        ELSE
            -- Show the current debug status (1n1t, rd F)
            w_display_data <= r_debug_display;
        END IF;
    END PROCESS;
    
    -- Assign status LEDs
    o_init_success_led <= r_init_done;
    o_file_found_led <= r_cmd_loaded; -- Use this LED to show the command is loaded

END rtl;