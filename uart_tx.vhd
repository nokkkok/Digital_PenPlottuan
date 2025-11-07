--------------------------------------------------------------------------------
-- File: uart_tx.vhd
-- Description: Simple UART Transmitter Module
-- Based on open-source UART designs (MIT License compatible)
-- 
-- Features:
--   - Configurable baud rate via generics
--   - Handshake interface (DIN_VLD/DIN_RDY)
--   - 8 data bits, 1 stop bit, no parity
--   - Tested and verified for FPGA implementation
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity uart_tx is
  generic (
    CLK_FREQ  : integer := 20_000_000;  -- System clock frequency in Hz
    BAUD_RATE : integer := 9600          -- UART baud rate
  );
  port (
    -- Clock and Reset
    CLK       : in  std_logic;           -- System clock
    RST       : in  std_logic;           -- Synchronous reset (active high)
    
    -- User Data Input Interface
    DIN       : in  std_logic_vector(7 downto 0);  -- Data to transmit
    DIN_VLD   : in  std_logic;           -- Data valid (start transmission)
    DIN_RDY   : out std_logic;           -- Ready for new data
    
    -- UART Interface
    UART_TXD  : out std_logic            -- Serial output
  );
end entity uart_tx;

architecture rtl of uart_tx is

  -- Calculate clocks per bit
  constant CLKS_PER_BIT : integer := CLK_FREQ / BAUD_RATE;
  
  -- FSM States
  type state_t is (
    S_IDLE,
    S_START,
    S_DATA,
    S_STOP
  );
  
  signal state      : state_t := S_IDLE;
  signal clk_count  : integer range 0 to CLKS_PER_BIT - 1 := 0;
  signal bit_index  : integer range 0 to 7 := 0;
  signal tx_data    : std_logic_vector(7 downto 0) := (others => '0');
  signal tx_line    : std_logic := '1';
  signal tx_ready   : std_logic := '1';
  
begin

  -- Connect outputs
  UART_TXD <= tx_line;
  DIN_RDY  <= tx_ready;
  
  -- Main UART TX Process
  process(CLK)
  begin
    if rising_edge(CLK) then
      if RST = '1' then
        -- Reset state
        state     <= S_IDLE;
        clk_count <= 0;
        bit_index <= 0;
        tx_line   <= '1';
        tx_ready  <= '1';
        
      else
        case state is
          
          -- IDLE: Wait for valid data
          when S_IDLE =>
            tx_line   <= '1';  -- Idle high
            tx_ready  <= '1';
            clk_count <= 0;
            bit_index <= 0;
            
            if DIN_VLD = '1' then
              tx_data  <= DIN;      -- Latch input data
              tx_ready <= '0';      -- Not ready anymore
              state    <= S_START;
            end if;
          
          -- START: Send start bit (0)
          when S_START =>
            tx_line <= '0';  -- Start bit
            
            if clk_count = CLKS_PER_BIT - 1 then
              clk_count <= 0;
              state     <= S_DATA;
            else
              clk_count <= clk_count + 1;
            end if;
          
          -- DATA: Send 8 data bits (LSB first)
          when S_DATA =>
            tx_line <= tx_data(bit_index);
            
            if clk_count = CLKS_PER_BIT - 1 then
              clk_count <= 0;
              
              if bit_index = 7 then
                bit_index <= 0;
                state     <= S_STOP;
              else
                bit_index <= bit_index + 1;
              end if;
            else
              clk_count <= clk_count + 1;
            end if;
          
          -- STOP: Send stop bit (1)
          when S_STOP =>
            tx_line <= '1';  -- Stop bit
            
            if clk_count = CLKS_PER_BIT - 1 then
              clk_count <= 0;
              tx_ready  <= '1';  -- Ready for next byte
              state     <= S_IDLE;
            else
              clk_count <= clk_count + 1;
            end if;
            
        end case;
      end if;
    end if;
  end process;

end architecture rtl;