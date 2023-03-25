require('dotenv').config({ path: './.env' });

const winston = require('winston');

const logger = winston.createLogger({
    level: 'info',
    format: winston.format.combine(
        winston.format.splat(),
        winston.format.simple()
    ),
    defaultMeta: {},
    transports: [
        new winston.transports.File({ filename: './logs/client/error.log', level: 'error' }),
        new winston.transports.File({ filename: './logs/client/combined.log' }),
    ],
})
if (process.env.NODE_ENV !== 'production') {
    logger.add(new winston.transports.Console({
        format: winston.format.combine(
            winston.format.colorize(),
            winston.format.splat(),
            winston.format.simple()
        ),
    }));
}

logger.exceptions.handle(
    new winston.transports.File({ filename: './logs/exceptions.log' })
)

module.exports = logger;