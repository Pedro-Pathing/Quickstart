package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.fail;

import com.networknt.schema.Error;
import com.networknt.schema.Schema;
import com.networknt.schema.SchemaRegistry;
import com.networknt.schema.SpecificationVersion;
import java.io.InputStream;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.yaml.snakeyaml.Yaml;
import tools.jackson.databind.JsonNode;
import tools.jackson.databind.ObjectMapper;

public class ConfigValidationTest {

  @Test
  public void validateConfigurations() {
    ObjectMapper mapper = new ObjectMapper();
    SchemaRegistry factory = SchemaRegistry.withDefaultDialect(SpecificationVersion.DRAFT_7);

    // Load schema
    InputStream schemaStream =
        getClass()
            .getResourceAsStream(
                "/org/firstinspires/ftc/teamcode/robot/config/generated/config-schema.json");
    assertNotNull(schemaStream, "Could not find config-schema.json in classpath resources");

    Schema schema = factory.getSchema(schemaStream);

    Yaml yaml = new Yaml();
    InputStream yamlStream =
        getClass().getResourceAsStream("/org/firstinspires/ftc/teamcode/robot/config/config.yaml");
    assertNotNull(yamlStream, "Could not find config file: config.yaml");

    Map<String, Object> loaded = yaml.load(yamlStream);
    assertNotNull(loaded, "Config file is empty: config.yaml");

    // Convert Map to JsonNode for validation
    JsonNode jsonNode = mapper.valueToTree(loaded);

    List<Error> errors = schema.validate(jsonNode);
    if (!errors.isEmpty()) {
      StringBuilder sb = new StringBuilder();
      sb.append("Validation failed for file: config.yaml\n");
      for (Error error : errors) {
        sb.append("  - ").append(error.getMessage()).append("\n");
      }
      fail(sb.toString());
    }
  }
}
