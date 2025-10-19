import { describe, test, expect, beforeEach, afterEach, jest } from '@jest/globals';
import { parseROSCommand } from '../../src/utils/ros';
import { ROSCommand } from '../../src/types/terminal';

describe('ROS Utilities', () => {
  let registry: ROSCommandRegistry;

  beforeEach(() => {
    registry = new ROSCommandRegistry();
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  describe('registerCommand', () => {
    test('should register basic ROS command', () => {
      const command: ROSCommand = {
        command: 'ros2 topic list',
        args: [],
        options: {},
        category: 'topic',
        description: 'List ROS topics',
        examples: ['ros2 topic list', 'ros2 topic list -t']
      };

      registry.registerCommand(command);

      const commands = registry.getCommands();
      expect(commands).toContain(command);
      expect(registry.getCommand('ros2 topic list')).toBe(command);
    });

    test('should register command with arguments', () => {
      const command: ROSCommand = {
        command: 'ros2 topic echo',
        args: ['topic_name'],
        options: {},
        category: 'topic',
        description: 'Echo topic messages',
        examples: ['ros2 topic echo /chatter']
      };

      registry.registerCommand(command);

      const foundCommand = registry.getCommand('ros2 topic echo');
      expect(foundCommand).toBe(command);
      expect(foundCommand.args).toEqual(['topic_name']);
    });

    test('should register command with options', () => {
      const command: ROSCommand = {
        command: 'ros2 node info',
        args: ['node_name'],
        options: {
          verbose: { type: 'boolean', description: 'Verbose output' },
          qos: { type: 'string', description: 'QoS profile' }
        },
        category: 'node',
        description: 'Get node information',
        examples: ['ros2 node info /talker', 'ros2 node info /talker --verbose']
      };

      registry.registerCommand(command);

      const foundCommand = registry.getCommand('ros2 node info');
      expect(foundCommand.options).toBeDefined();
      expect(foundCommand.options.verbose).toBeDefined();
    });

    test('should validate command before registration', () => {
      const invalidCommand = {
        command: '',
        args: [],
        options: {},
        category: 'topic',
        description: 'Invalid command'
      } as ROSCommand;

      expect(() => {
        registry.registerCommand(invalidCommand);
      }).toThrow('Command cannot be empty');
    });

    test('should prevent duplicate command registration', () => {
      const command: ROSCommand = {
        command: 'ros2 topic list',
        args: [],
        options: {},
        category: 'topic',
        description: 'List topics'
      };

      registry.registerCommand(command);

      expect(() => {
        registry.registerCommand(command);
      }).toThrow('Command already registered: ros2 topic list');
    });
  });

  describe('getCommandsByCategory', () => {
    test('should return commands filtered by category', () => {
      const topicCommand: ROSCommand = {
        command: 'ros2 topic list',
        args: [],
        options: {},
        category: 'topic',
        description: 'List topics'
      };

      const nodeCommand: ROSCommand = {
        command: 'ros2 node list',
        args: [],
        options: {},
        category: 'node',
        description: 'List nodes'
      };

      registry.registerCommand(topicCommand);
      registry.registerCommand(nodeCommand);

      const topicCommands = registry.getCommandsByCategory('topic');
      const nodeCommands = registry.getCommandsByCategory('node');

      expect(topicCommands).toHaveLength(1);
      expect(topicCommands[0]).toBe(topicCommand);
      expect(nodeCommands).toHaveLength(1);
      expect(nodeCommands[0]).toBe(nodeCommand);
    });

    test('should return empty array for non-existent category', () => {
      const commands = registry.getCommandsByCategory('nonexistent');
      
      expect(commands).toEqual([]);
    });
  });

  describe('searchCommands', () => {
    test('should search commands by keyword', () => {
      const commands = [
        {
          command: 'ros2 topic list',
          args: [],
          options: {},
          category: 'topic',
          description: 'List all ROS topics'
        },
        {
          command: 'ros2 topic echo',
          args: ['topic_name'],
          options: {},
          category: 'topic',
          description: 'Echo messages from a topic'
        },
        {
          command: 'ros2 node list',
          args: [],
          options: {},
          category: 'node',
          description: 'List all ROS nodes'
        }
      ] as ROSCommand[];

      commands.forEach(cmd => registry.registerCommand(cmd));

      const topicResults = registry.searchCommands('topic');
      const listResults = registry.searchCommands('list');
      const echoResults = registry.searchCommands('echo');

      expect(topicResults).toHaveLength(2);
      expect(listResults).toHaveLength(2);
      expect(echoResults).toHaveLength(1);
    });

    test('should search commands by description', () => {
      const command: ROSCommand = {
        command: 'ros2 service call',
        args: ['service_name', 'service_type'],
        options: {},
        category: 'service',
        description: 'Call a ROS service with specific parameters'
      };

      registry.registerCommand(command);

      const results = registry.searchCommands('call service');
      expect(results).toHaveLength(1);
      expect(results[0]).toBe(command);
    });

    test('should handle case-insensitive search', () => {
      const command: ROSCommand = {
        command: 'ros2 TOPIC LIST',
        args: [],
        options: {},
        category: 'topic',
        description: 'List topics'
      };

      registry.registerCommand(command);

      const results = registry.searchCommands('topic list');
      expect(results).toHaveLength(1);
    });
  });

  describe('getCommandExamples', () => {
    test('should return formatted command examples', () => {
      const command: ROSCommand = {
        command: 'ros2 topic echo',
        args: ['topic_name'],
        options: {},
        category: 'topic',
        description: 'Echo topic messages',
        examples: [
          'ros2 topic echo /chatter',
          'ros2 topic echo /chatter --once',
          'ros2 topic echo /chatter --csv'
        ]
      };

      registry.registerCommand(command);

      const examples = registry.getCommandExamples('ros2 topic echo');
      expect(examples).toHaveLength(3);
      expect(examples[0]).toBe('ros2 topic echo /chatter');
    });

    test('should return empty array for command without examples', () => {
      const command: ROSCommand = {
        command: 'ros2 topic hz',
        args: ['topic_name'],
        options: {},
        category: 'topic',
        description: 'Get topic frequency'
      };

      registry.registerCommand(command);

      const examples = registry.getCommandExamples('ros2 topic hz');
      expect(examples).toEqual([]);
    });
  });

  describe('validateCommand', () => {
    test('should validate command arguments', () => {
      const command: ROSCommand = {
        command: 'ros2 topic echo',
        args: ['topic_name'],
        options: {},
        category: 'topic',
        description: 'Echo topic messages'
      };

      registry.registerCommand(command);

      const validResult = registry.validateCommand('ros2 topic echo', ['/chatter']);
      const invalidResult = registry.validateCommand('ros2 topic echo', []);

      expect(validResult.valid).toBe(true);
      expect(invalidResult.valid).toBe(false);
      expect(invalidResult.errors).toContain('Missing required argument: topic_name');
    });

    test('should validate command options', () => {
      const command: ROSCommand = {
        command: 'ros2 node info',
        args: ['node_name'],
        options: {
          verbose: { type: 'boolean', description: 'Verbose output' },
          qos: { type: 'string', description: 'QoS profile' }
        },
        category: 'node',
        description: 'Get node information'
      };

      registry.registerCommand(command);

      const result = registry.validateCommand('ros2 node info', ['/talker'], { verbose: true, qos: 'reliable' });
      expect(result.valid).toBe(true);

      const invalidResult = registry.validateCommand('ros2 node info', ['/talker'], { invalid: 'option' });
      expect(invalidResult.valid).toBe(false);
      expect(invalidResult.errors).toContain('Unknown option: invalid');
    });

    test('should validate option types', () => {
      const command: ROSCommand = {
        command: 'ros2 param set',
        args: ['node_name', 'param_name', 'param_value'],
        options: {
        use_sim_time: { type: 'boolean', description: 'Use simulation time' }
        },
        category: 'parameter',
        description: 'Set parameter'
      };

      registry.registerCommand(command);

      const validResult = registry.validateCommand('ros2 param set', ['/node', 'param', 'value'], { use_sim_time: true });
      const invalidResult = registry.validateCommand('ros2 param set', ['/node', 'param', 'value'], { use_sim_time: 'not_boolean' });

      expect(validResult.valid).toBe(true);
      expect(invalidResult.valid).toBe(false);
      expect(invalidResult.errors).toContain('Option use_sim_time must be a boolean');
    });
  });

  describe('executeROSCommand', () => {
    test('should execute registered ROS command', async () => {
      const command: ROSCommand = {
        command: 'ros2 topic list',
        args: [],
        options: {},
        category: 'topic',
        description: 'List topics',
        examples: ['ros2 topic list']
      };

      registry.registerCommand(command);

      const result = await executeROSCommand('ros2 topic list', []);
      
      expect(result.success).toBe(true);
      expect(result.output).toBeDefined();
      expect(result.command).toBe('ros2 topic list');
    });

    test('should execute command with arguments', async () => {
      const command: ROSCommand = {
        command: 'ros2 topic echo',
        args: ['topic_name'],
        options: {},
        category: 'topic',
        description: 'Echo topic messages',
        examples: ['ros2 topic echo /chatter']
      };

      registry.registerCommand(command);

      const result = await executeROSCommand('ros2 topic echo', ['/chatter']);
      
      expect(result.success).toBe(true);
      expect(result.command).toBe('ros2 topic echo /chatter');
    });

    test('should handle command execution errors', async () => {
      const command: ROSCommand = {
        command: 'ros2 invalid command',
        args: [],
        options: {},
        category: 'topic',
        description: 'Invalid command'
      };

      registry.registerCommand(command);

      const result = await executeROSCommand('ros2 invalid command', []);
      
      expect(result.success).toBe(false);
      expect(result.error).toBeDefined();
      expect(result.error).toContain('Unknown command');
    });

    test('should parse command output', async () => {
      const command: ROSCommand = {
        command: 'ros2 topic list',
        args: [],
        options: {},
        category: 'topic',
        description: 'List topics',
        parser: (output: string) => output.split('\n').filter(line => line.trim())
      };

      registry.registerCommand(command);

      const result = await executeROSCommand('ros2 topic list', []);
      
      expect(result.success).toBe(true);
      expect(result.parsedOutput).toBeDefined();
      expect(Array.isArray(result.parsedOutput)).toBe(true);
    });
  });

  describe('parseROSOutput', () => {
    test('should parse topic list output', () => {
      const output = `/chatter
/parameter_events
/rosout`;

      const parsed = parseROSOutput(output, 'topic_list');
      
      expect(parsed).toHaveLength(3);
      expect(parsed).toContain('/chatter');
      expect(parsed).toContain('/parameter_events');
      expect(parsed).toContain('/rosout');
    });

    test('should parse node list output', () => {
      const output = `/talker
/listener
/rosout`;

      const parsed = parseROSOutput(output, 'node_list');
      
      expect(parsed).toHaveLength(3);
      expect(parsed).toContain('/talker');
      expect(parsed).toContain('/listener');
    });

    test('should parse service list output', () => {
      const output = `/add_two_ints
/parameter_events
/rosout`;

      const parsed = parseROSOutput(output, 'service_list');
      
      expect(parsed).toHaveLength(3);
      expect(parsed).toContain('/add_two_ints');
    });

    test('should parse parameter list output', () => {
      const output = `use_sim_time
qos_profile
debug_mode`;

      const parsed = parseROSOutput(output, 'parameter_list');
      
      expect(parsed).toHaveLength(3);
      expect(parsed).toContain('use_sim_time');
    });

    test('should handle empty output', () => {
      const parsed = parseROSOutput('', 'topic_list');
      
      expect(parsed).toEqual([]);
    });

    test('should handle malformed output gracefully', () => {
      const malformedOutput = `  /chatter  
  
  /rosout  `;

      const parsed = parseROSOutput(malformedOutput, 'topic_list');
      
      expect(parsed).toHaveLength(2);
      expect(parsed).toContain('/chatter');
      expect(parsed).toContain('/rosout');
    });
  });

  describe('Command Categories', () => {
    test('should organize commands by category', () => {
      const commands = [
        { command: 'ros2 topic list', category: 'topic' },
        { command: 'ros2 topic echo', category: 'topic' },
        { command: 'ros2 node list', category: 'node' },
        { command: 'ros2 node info', category: 'node' },
        { command: 'ros2 service list', category: 'service' },
        { command: 'ros2 param list', category: 'parameter' },
        { command: 'ros2 launch', category: 'launch' }
      ] as ROSCommand[];

      commands.forEach(cmd => registry.registerCommand(cmd));

      const categories = registry.getCategories();
      expect(categories).toContain('topic');
      expect(categories).toContain('node');
      expect(categories).toContain('service');
      expect(categories).toContain('parameter');
      expect(categories).toContain('launch');

      const topicCommands = registry.getCommandsByCategory('topic');
      expect(topicCommands).toHaveLength(2);
    });

    test('should provide category descriptions', () => {
      const categories = registry.getCategories();
      const categoryInfo = registry.getCategoryInfo('topic');
      
      expect(categoryInfo).toBeDefined();
      expect(categoryInfo.name).toBe('topic');
      expect(categoryInfo.description).toContain('ROS topics');
    });
  });

  describe('Command History and Favorites', () => {
    test('should track command execution history', () => {
      const command: ROSCommand = {
        command: 'ros2 topic list',
        args: [],
        options: {},
        category: 'topic',
        description: 'List topics'
      };

      registry.registerCommand(command);
      registry.recordCommandExecution('ros2 topic list');
      registry.recordCommandExecution('ros2 topic list');

      const history = registry.getCommandHistory();
      expect(history).toHaveLength(2);
      expect(history[0].command).toBe('ros2 topic list');
      expect(history[0].count).toBe(2);
    });

    test('should manage favorite commands', () => {
      const command: ROSCommand = {
        command: 'ros2 topic list',
        args: [],
        options: {},
        category: 'topic',
        description: 'List topics'
      };

      registry.registerCommand(command);
      registry.addFavorite('ros2 topic list');

      const favorites = registry.getFavorites();
      expect(favorites).toContain('ros2 topic list');

      registry.removeFavorite('ros2 topic list');
      expect(registry.getFavorites()).not.toContain('ros2 topic list');
    });

    test('should suggest commands based on usage', () => {
      const commands = [
        { command: 'ros2 topic list', category: 'topic' },
        { command: 'ros2 topic echo', category: 'topic' },
        { command: 'ros2 node list', category: 'node' }
      ] as ROSCommand[];

      commands.forEach(cmd => registry.registerCommand(cmd));
      
      // Record usage
      registry.recordCommandExecution('ros2 topic list');
      registry.recordCommandExecution('ros2 topic list');
      registry.recordCommandExecution('ros2 topic echo');

      const suggestions = registry.getSuggestions('ros2 topic');
      expect(suggestions[0].command).toBe('ros2 topic list'); // Most used
      expect(suggestions[1].command).toBe('ros2 topic echo');
    });
  });

  describe('Performance and Caching', () => {
    test('should cache command validation results', () => {
      const command: ROSCommand = {
        command: 'ros2 topic list',
        args: [],
        options: {},
        category: 'topic',
        description: 'List topics'
      };

      registry.registerCommand(command);

      const startTime = performance.now();
      const result1 = registry.validateCommand('ros2 topic list', []);
      const midTime = performance.now();
      const result2 = registry.validateCommand('ros2 topic list', []);
      const endTime = performance.now();

      expect(result1.valid).toBe(true);
      expect(result2.valid).toBe(true);
      expect(endTime - midTime).toBeLessThan(midTime - startTime); // Second validation should be faster
    });

    test('should handle large command sets efficiently', () => {
      // Register many commands
      for (let i = 0; i < 1000; i++) {
        registry.registerCommand({
          command: `ros2 custom${i}`,
          args: [],
          options: {},
          category: 'custom',
          description: `Custom command ${i}`
        });
      }

      const startTime = performance.now();
      const results = registry.searchCommands('custom');
      const endTime = performance.now();

      expect(results).toHaveLength(1000);
      expect(endTime - startTime).toBeLessThan(100); // Should search in less than 100ms
    });
  });

  describe('Error Handling and Edge Cases', () => {
    test('should handle malformed command gracefully', () => {
      const malformedCommand = {
        command: null,
        args: [],
        options: {},
        category: 'topic',
        description: 'Malformed command'
      } as any;

      expect(() => {
        registry.registerCommand(malformedCommand);
      }).toThrow('Invalid command format');
    });

    test('should handle circular references in options', () => {
      const circularOptions: any = { ref: {} };
      circularOptions.ref.self = circularOptions;

      const command: ROSCommand = {
        command: 'ros2 test',
        args: [],
        options: circularOptions,
        category: 'test',
        description: 'Test command'
      };

      expect(() => {
        registry.registerCommand(command);
      }).not.toThrow();
    });

    test('should handle very long command descriptions', () => {
      const longDescription = 'a'.repeat(10000);
      const command: ROSCommand = {
        command: 'ros2 long',
        args: [],
        options: {},
        category: 'test',
        description: longDescription
      };

      registry.registerCommand(command);
      const foundCommand = registry.getCommand('ros2 long');
      expect(foundCommand?.description).toBe(longDescription);
    });

    test('should handle concurrent command registration', async () => {
      const commands = Array.from({ length: 100 }, (_, i) => ({
        command: `ros2 concurrent${i}`,
        args: [],
        options: {},
        category: 'test',
        description: `Concurrent command ${i}`
      }));

      // Register commands concurrently
      await Promise.all(commands.map(cmd => registry.registerCommand(cmd)));
      const allCommands = registry.getCommands();
      expect(allCommands).toHaveLength(100);
    });
  });
});
    });
  });
});