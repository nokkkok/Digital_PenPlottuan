library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity stepper_control_top is
    port (
        -- System
        i_clk   : in  std_logic; -- 20 MHz Clock
        i_rst   : in  std_logic; -- PB1 (P45)
        
        -- UART
        i_rxd   : in  std_logic; -- RXD (P23)
        
        -- Inputs
        i_display_btn : in  std_logic; -- PB3 (P47)
        i_stepper_driver_enable : in std_logic; -- SW0 (P66)
        
        -- *** HOMING INPUTS ***
        i_x_endstop : in  std_logic;
        i_y_endstop : in  std_logic;
        
        -- 7-Segment Display
        o_seg_anodes   : out std_logic_vector(1 downto 0);
        o_seg_cathodes : out std_logic_vector(6 downto 0);
        
        -- Motor Outputs
        o_stepper_enable : out std_logic; -- P24 (Active-Low)
        o_stepper_x_dir  : out std_logic; -- P22
        o_stepper_x_step : out std_logic; -- P17
        o_stepper_y_dir  : out std_logic; -- P12
        o_stepper_y_step : out std_logic; -- P10
        o_servo_pwm    : out std_logic; -- P15
        
        -- Debug LEDs
        o_heartbeat_led : out std_logic; -- L0 (P82)
        o_rx_activity_led : out std_logic; -- L2 (P80)
        o_busy_led : out std_logic      -- L3 (P79)
    );
end entity stepper_control_top;

architecture rtl of stepper_control_top is

    --==================================================================
    -- UART PACKET PARSER SIGNALS
    --==================================================================
    signal w_x_steps_uart    : std_logic_vector(23 downto 0);
    signal w_y_steps_uart    : std_logic_vector(23 downto 0);
    signal w_control_uart    : std_logic_vector(7 downto 0);
    signal w_new_command   : std_logic;
    
    --==================================================================
    -- 7-SEGMENT DISPLAY SIGNALS
    --==================================================================
    signal r_x_data_display  : std_logic_vector(23 downto 0) := (others => '0');
    signal r_y_data_display  : std_logic_vector(23 downto 0) := (others => '0');
    signal r_control_display : std_logic_vector(7 downto 0)  := (others => '0');
    signal r_btn_sync        : std_logic_vector(1 downto 0);
    signal r_display_index   : integer range 0 to 6 := 0;
    signal w_byte_to_show    : std_logic_vector(7 downto 0);
    
    --==================================================================
    -- STEPPER MOTOR SIGNALS
    --==================================================================
    
    -- 500 Hz Stepper Pulse Generator
    constant C_STEP_HALF_PERIOD : natural := 19999; -- 20,000,000 / 20,000 = 1000 Hz / 2 = 500 Hz
    signal r_stepper_x_counter : natural range 0 to C_STEP_HALF_PERIOD := 0;
    signal r_stepper_y_counter : natural range 0 to C_STEP_HALF_PERIOD := 0;
    
    -- 1-cycle pulse for FSM internal step counting
    signal w_stepper_x_pulse : std_logic := '0'; 
    signal w_stepper_y_pulse : std_logic := '0';
    
    -- *** NEW: 50% Duty Cycle clock for actual motor output ***
    signal r_stepper_x_clk_out : std_logic := '0';
    signal r_stepper_y_clk_out : std_logic := '0';
    
    -- Step Counters (to track remaining steps)
    signal r_x_steps_remaining : unsigned(23 downto 0) := (others => '0');
    signal r_y_steps_remaining : unsigned(23 downto 0) := (others => '0');
    
    -- State Machines (with Homing states)
    type t_stepper_state is (s_homing_y, s_homing_x, s_idle, s_stepping);
    signal r_x_stepper_state : t_stepper_state := s_homing_y;
    signal r_y_stepper_state : t_stepper_state := s_homing_y;
    
    -- Endstop Synchronizers
    signal r_x_endstop_sync : std_logic_vector(1 downto 0);
    signal r_y_endstop_sync : std_logic_vector(1 downto 0);
    signal w_x_endstop_synced : std_logic;
    signal w_y_endstop_synced : std_logic;
    
    -- Synchronizer for the enable switch
    signal r_enable_sync : std_logic_vector(1 downto 0);
    signal w_stepper_enable_synced : std_logic;
    
    --==================================================================
    -- SERVO MOTOR SIGNALS
    --==================================================================
    constant C_CLK_FREQ_HZ : integer := 20_000_000;
    constant C_SERVO_PERIOD_CYCLES : integer := (C_CLK_FREQ_HZ / 1000) * 20;
    constant C_SERVO_MIN_PULSE_CYCLES : integer := C_CLK_FREQ_HZ / 1000;
    constant C_SERVO_90DEG_PULSE_CYCLES : integer := (C_CLK_FREQ_HZ * 3) / 2000;
    
    signal r_servo_period_counter : integer range 0 to C_SERVO_PERIOD_CYCLES - 1 := 0;
    signal w_servo_target_width   : integer range C_SERVO_MIN_PULSE_CYCLES to C_SERVO_90DEG_PULSE_CYCLES;
    signal w_servo_position_select : std_logic;
    
    --==================================================================
    -- DEBUG LED SIGNALS
    --==================================================================
    signal r_heartbeat_counter : unsigned(23 downto 0) := (others => '0');
    signal w_rx_activity_from_parser : std_logic;
    signal r_busy_led : std_logic := '0';
    
    --==================================================================
    -- COMPONENT DECLARATIONS
    --==================================================================
    
    component packet_parser is
        generic ( g_CLKS_PER_BIT : integer := 2083 );
        port (
            i_clk   : in  std_logic;
            i_rst   : in  std_logic;
            i_rxd   : in  std_logic;
            o_x_steps : out std_logic_vector(23 downto 0);
            o_y_steps : out std_logic_vector(23 downto 0);
            o_control : out std_logic_vector(7 downto 0);
            o_new_command_valid : out std_logic;
            o_rx_activity_led : out std_logic
        );
    end component;
    
    component display_multiplexer is
        port (
            i_clk         : in  std_logic;
            i_byte_to_show : in  std_logic_vector(7 downto 0);
            o_seg_anodes   : out std_logic_vector(1 downto 0);
            o_seg_cathodes : out std_logic_vector(6 downto 0)
        );
    end component;

begin

    --==================================================================
    -- INSTANTIATE SUB-MODULES
    --==================================================================

    u_parser : packet_parser
        generic map ( g_CLKS_PER_BIT => 2083 ) 
        port map (
            i_clk   => i_clk,
            i_rst   => i_rst,
            i_rxd   => i_rxd,
            o_x_steps => w_x_steps_uart,
            o_y_steps => w_y_steps_uart,
            o_control => w_control_uart,
            o_new_command_valid => w_new_command,
            o_rx_activity_led => w_rx_activity_from_parser
        );

    u_display : display_multiplexer
        port map (
            i_clk          => i_clk,
            i_byte_to_show => w_byte_to_show,
            o_seg_anodes   => o_seg_anodes,
            o_seg_cathodes => o_seg_cathodes
        );

    --==================================================================
    -- INPUT SYNCHRONIZATION (for Endstops, Button, and Enable Switch)
    --==================================================================
    p_input_sync : process(i_clk)
    begin
        if (rising_edge(i_clk)) then
            -- Synchronize async endstop inputs
            r_x_endstop_sync(0) <= i_x_endstop;
            r_x_endstop_sync(1) <= r_x_endstop_sync(0);
            
            r_y_endstop_sync(0) <= i_y_endstop;
            r_y_endstop_sync(1) <= r_y_endstop_sync(0);
            
            -- Synchronize button for display
            r_btn_sync(0) <= i_display_btn;
            r_btn_sync(1) <= r_btn_sync(0);
            
            -- Synchronize the enable switch
            r_enable_sync(0) <= i_stepper_driver_enable;
            r_enable_sync(1) <= r_enable_sync(0);
        end if;
    end process;
    
    -- Create wires for the synced signals
    w_x_endstop_synced <= r_x_endstop_sync(1);
    w_y_endstop_synced <= r_y_endstop_sync(1);
    w_stepper_enable_synced <= r_enable_sync(1);

    --==================================================================
    -- 7-SEGMENT DISPLAY LOGIC
    --==================================================================
    
    p_display_control : process (i_clk)
        variable v_btn_pulse : std_logic;
    begin
        if (rising_edge(i_clk)) then
            -- Button edge detection (using pre-synced signals)
            if (r_btn_sync(0) = '1' and r_btn_sync(1) = '0') then
                v_btn_pulse := '1';
            else
                v_btn_pulse := '0';
            end if;

            if (i_rst = '1') then
                r_display_index <= 0;
                r_x_data_display <= (others => '0');
                r_y_data_display <= (others => '0');
                r_control_display <= (others => '0');
            else
                -- Latch new data when it arrives
                if (w_new_command = '1') then
                    r_x_data_display  <= w_x_steps_uart;
                    r_y_data_display  <= w_y_steps_uart;
                    r_control_display <= w_control_uart;
                end if;
                
                -- Cycle display on button press
                if (v_btn_pulse = '1') then
                    if (r_display_index = 6) then
                        r_display_index <= 0;
                    else
                        r_display_index <= r_display_index + 1;
                    end if;
                end if;
            end if;
        end if;
    end process;

    p_byte_mux : process (r_display_index, r_x_data_display, r_y_data_display, r_control_display)
    begin
        case r_display_index is
            when 0 => w_byte_to_show <= r_x_data_display(23 downto 16); -- X-High
            when 1 => w_byte_to_show <= r_x_data_display(15 downto 8);  -- X-Mid
            when 2 => w_byte_to_show <= r_x_data_display(7 downto 0);   -- X-Low
            when 3 => w_byte_to_show <= r_y_data_display(23 downto 16); -- Y-High
            when 4 => w_byte_to_show <= r_y_data_display(15 downto 8);  -- Y-Mid
            when 5 => w_byte_to_show <= r_y_data_display(7 downto 0);   -- Y-Low
            when 6 => w_byte_to_show <= r_control_display;              -- Control
            when others => w_byte_to_show <= x"EE";                     -- Error
        end case;
    end process p_byte_mux;

    --==================================================================
    -- STEPPER MOTOR LOGIC
    --==================================================================

    -- Stepper Enable (from SW0) - ACTIVE-LOW OUTPUT
    -- Use the clean, synced signal
    o_stepper_enable <= not w_stepper_enable_synced;
    
    -- X-Axis Stepper FSM (Left Motor)
    p_stepper_x_fsm : process(i_clk, i_rst)
    begin
        if (i_rst = '1') then
            r_x_stepper_state <= s_homing_y; -- Start in Homing state
            r_x_steps_remaining <= (others => '0');
            o_stepper_x_dir <= '1'; -- Default
            r_stepper_x_counter <= 0;
            w_stepper_x_pulse <= '0';
            r_stepper_x_clk_out <= '0'; -- *** NEW: Reset the output clock
            
        elsif (rising_edge(i_clk)) then
            
            w_stepper_x_pulse <= '0'; -- Default to 0
            
            -- 500 Hz Pulse/Clock Generator (runs when stepping OR homing)
            if (r_x_stepper_state = s_stepping or r_x_stepper_state = s_homing_y or r_x_stepper_state = s_homing_x) then
                if (r_stepper_x_counter = C_STEP_HALF_PERIOD) then
                    r_stepper_x_counter <= 0;
                    w_stepper_x_pulse <= '1'; -- Generate a 1-cycle pulse (for FSM step counting)
                    r_stepper_x_clk_out <= not r_stepper_x_clk_out; -- *** NEW: Toggle the 50% duty cycle clock
                else
                    r_stepper_x_counter <= r_stepper_x_counter + 1;
                end if;
            else
                r_stepper_x_counter <= 0; -- Hold counter at 0 when idle
                r_stepper_x_clk_out <= '0'; -- *** NEW: Hold output clock low when idle
            end if;
            
            -- FSM Logic
            case r_x_stepper_state is
                
                when s_homing_y =>
                    -- Phase 1: Move Y to 0
                    -- Left motor (X) moves Clockwise
                    o_stepper_x_dir <= '1'; 
                    if (w_y_endstop_synced = '1') then -- Check for active-high endstop
                        r_x_stepper_state <= s_homing_x;
                    end if;
                    
                when s_homing_x =>
                    -- Phase 2: Move X to 0
                    -- Left motor (X) moves Anti-Clockwise
                    o_stepper_x_dir <= '0';
                    if (w_x_endstop_synced = '1') then -- Check for active-high endstop
                        r_x_stepper_state <= s_idle; -- Homing complete
                    end if;

                when s_idle =>
                    -- Ready for UART commands
                    if (w_new_command = '1') then
                        if (unsigned(w_x_steps_uart) > 0) then
                            r_x_steps_remaining <= unsigned(w_x_steps_uart);
                            o_stepper_x_dir <= w_control_uart(1); -- Bit 1 = X-Dir
                            r_x_stepper_state <= s_stepping;
                        end if;
                    end if;
                    
                when s_stepping =>
                    -- Running a UART command
                    -- This logic uses the 1-cycle 'w_stepper_x_pulse' to count
                    if (w_stepper_x_pulse = '1') then 
                        if (r_x_steps_remaining = 1) then
                            r_x_steps_remaining <= (others => '0');
                            r_x_stepper_state <= s_idle;
                        else
                            r_x_steps_remaining <= r_x_steps_remaining - 1;
                        end if;
                    end if;
                    
            end case;
        end if;
    end process;
    
    -- Y-Axis Stepper FSM (Right Motor)
    p_stepper_y_fsm : process(i_clk, i_rst)
    begin
        if (i_rst = '1') then
            r_y_stepper_state <= s_homing_y; -- Start in Homing state
            r_y_steps_remaining <= (others => '0');
            o_stepper_y_dir <= '1'; -- Default
            r_stepper_y_counter <= 0;
            w_stepper_y_pulse <= '0';
            r_stepper_y_clk_out <= '0'; -- *** NEW: Reset the output clock
            
        elsif (rising_edge(i_clk)) then
            
            w_stepper_y_pulse <= '0'; -- Default to 0
            
            -- 500 Hz Pulse/Clock Generator (runs when stepping OR homing)
            if (r_y_stepper_state = s_stepping or r_y_stepper_state = s_homing_y or r_y_stepper_state = s_homing_x) then
                if (r_stepper_y_counter = C_STEP_HALF_PERIOD) then
                    r_stepper_y_counter <= 0; 
                    w_stepper_y_pulse <= '1'; -- Generate a 1-cycle pulse (for FSM step counting)
                    r_stepper_y_clk_out <= not r_stepper_y_clk_out; -- *** NEW: Toggle the 50% duty cycle clock
                else
                    r_stepper_y_counter <= r_stepper_y_counter + 1; 
                end if;
            else
                r_stepper_y_counter <= 0; 
                r_stepper_y_clk_out <= '0'; -- *** NEW: Hold output clock low when idle
            end if;
            
            case r_y_stepper_state is
            
                when s_homing_y =>
                    -- Phase 1: Move Y to 0
                    -- Right motor (Y) moves Anti-Clockwise
                    o_stepper_y_dir <= '0';
                    if (w_y_endstop_synced = '1') then -- Check for active-high endstop
                        r_y_stepper_state <= s_homing_x;
                    end if;
                    
                when s_homing_x =>
                    -- Phase 2: Move X to 0
                    -- Right motor (Y) moves Anti-Clockwise
                    o_stepper_y_dir <= '0';
                    if (w_x_endstop_synced = '1') then -- Check for active-high endstop
                        r_y_stepper_state <= s_idle; -- Homing complete
                    end if;
                
                when s_idle =>
                    -- Ready for UART commands
                    if (w_new_command = '1') then
                        if (unsigned(w_y_steps_uart) > 0) then
                            r_y_steps_remaining <= unsigned(w_y_steps_uart);
                            o_stepper_y_dir <= w_control_uart(2); -- Bit 2 = Y-Dir
                            r_y_stepper_state <= s_stepping;
                        end if;
                    end if;
                    
                when s_stepping =>
                    -- Running a UART command
                    -- This logic uses the 1-cycle 'w_stepper_y_pulse' to count
                    if (w_stepper_y_pulse = '1') then
                        if (r_y_steps_remaining = 1) then
                            r_y_steps_remaining <= (others => '0');
                            r_y_stepper_state <= s_idle;
                        else
                            r_y_steps_remaining <= r_y_steps_remaining - 1;
                        end if;
                    end if;
                    
            end case;
        end if;
    end process;

    -- Final Step Output (only if enabled)
    -- *** Use the new 50% duty cycle clock signal ***
    o_stepper_x_step <= r_stepper_x_clk_out and w_stepper_enable_synced;
    o_stepper_y_step <= r_stepper_y_clk_out and w_stepper_enable_synced;
    
    --==================================================================
    -- SERVO MOTOR LOGIC
    --==================================================================
    
    w_servo_position_select <= w_control_uart(0);

    p_servo_period_counter : process(i_clk, i_rst)
    begin
        if i_rst = '1' then
            r_servo_period_counter <= 0;
        elsif rising_edge(i_clk) then
            if r_servo_period_counter = C_SERVO_PERIOD_CYCLES - 1 then
                r_servo_period_counter <= 0;
            else
                r_servo_period_counter <= r_servo_period_counter + 1;
            end if;
        end if;
    end process;

    p_servo_pulse_width : process(w_servo_position_select)
    begin
        if w_servo_position_select = '0' then
            w_servo_target_width <= C_SERVO_MIN_PULSE_CYCLES;
        else
            w_servo_target_width <= C_SERVO_90DEG_PULSE_CYCLES;
        end if;
    end process;

    p_servo_output : process(r_servo_period_counter, w_servo_target_width)
    begin
        if r_servo_period_counter < w_servo_target_width then
            o_servo_pwm <= '1';
        else
            o_servo_pwm <= '0';
        end if;
    end process;

--------------------------------------------------------------------
-- DEBUG LED LOGIC
--==================================================================
    
    -- L0: Heartbeat LED (Active High)
    p_heartbeat : process(i_clk)
    begin
        if (rising_edge(i_clk)) then
            r_heartbeat_counter <= r_heartbeat_counter + 1;
        end if;
    end process;
    o_heartbeat_led <= r_heartbeat_counter(23);

    -- L2: RX Activity LED
    o_rx_activity_led <= w_rx_activity_from_parser;

    -- L3: Busy LED (Active High)
    -- MODIFIED to be active during HOMING or STEPPING
    p_busy_led : process(i_clk, i_rst)
    begin
        if (i_rst = '1') then
            r_busy_led <= '1'; -- We are busy homing on reset
        elsif (rising_edge(i_clk)) then
            -- Busy if EITHER FSM is not in the idle state
            if (r_x_stepper_state = s_idle and r_y_stepper_state = s_idle) then
                r_busy_led <= '0'; -- Both are idle, so we are not busy
            else
                r_busy_led <= '1'; -- One or both are homing/stepping
            end if;
        end if;
    end process;
    o_busy_led <= r_busy_led;

end architecture rtl;