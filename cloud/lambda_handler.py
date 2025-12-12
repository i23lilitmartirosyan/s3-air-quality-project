import json
import boto3
import datetime
from datetime import timezone, timedelta
import base64
from botocore.exceptions import ClientError

s3 = boto3.client("s3")
BUCKET_NAME = "fromesptos3"

def lambda_handler(event, context):
    try:
        # Decode JSON from API Gateway
        if "body" in event:
            if event.get("isBase64Encoded", False):
                body = base64.b64decode(event["body"])
                data = json.loads(body)
            else:
                data = json.loads(event["body"])
        else:
            data = event
        
        device_id = data.get("deviceId", "UnknownDevice")
        filename = f"{device_id}/data.json"
        
        # Add timestamp in Armenian timezone (UTC+4)
        armenian_tz = timezone(timedelta(hours=4))
        data["timestamp"] = datetime.datetime.now(armenian_tz).isoformat()
        
        # Load existing JSON file from S3
        try:
            response = s3.get_object(Bucket=BUCKET_NAME, Key=filename)
            existing = json.loads(response["Body"].read())
        except ClientError:
            existing = {"readings": []}
        
        # Append the reading
        existing["readings"].append(data)
        
        # Save updated JSON back to S3
        s3.put_object(
            Bucket=BUCKET_NAME,
            Key=filename,
            Body=json.dumps(existing, indent=2),
            ContentType="application/json"
        )
        
        return {
            "statusCode": 200,
            "body": json.dumps({
                "message": "Data appended successfully",
                "file": filename
            })
        }
        
    except Exception as e:
        print("Error:", str(e))
        return {
            "statusCode": 500,
            "body": json.dumps({"error": str(e)})
        }